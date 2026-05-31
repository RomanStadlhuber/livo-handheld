/// @file
/// @ingroup bundle_adjustment
#include <mapping/BundleAdjustment.hpp>
#include <mapping/logging.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <numeric>
#include <algorithm>
#include <limits>
#include <cmath>
#include <pthread.h>
#include <semaphore.h>

SETUP_LOGS(DEBUG, "BundleAdjustment");

namespace mapping
{
    namespace
    {
        // stiffness applied to edges between fixed reference nodes to emulate
        // multi-fixed-node behavior that Open3D's PGO does not support natively
        constexpr double FIXED_NODE_INFO_SCALE = 1e8;
        // minimum number of required active submaps before building & optimizing a graph
        constexpr std::size_t MIN_ACTIVE_SUBMAPS = 2;

    } // namespace

    BundleAdjustment::BundleAdjustment(const MappingConfig &config) : config_(config)
    {
        sem_init(&workSemaphore_, 0, 0);

        const int totalCpus = static_cast<int>(std::thread::hardware_concurrency());
        const int baCpus = std::max(1, totalCpus / 2);
        backgroundArena_ = std::make_unique<tbb::task_arena>(baCpus, 1, tbb::task_arena::priority::low);
        LOG(INFO, "BA arena: " << baCpus << "/" << totalCpus << " threads, low priority");

        LOG(INFO, "BundleAdjustment initialized");
    }

    BundleAdjustment::~BundleAdjustment()
    {
        if (optimizationThread_.joinable())
            stopOptimizationWorker();
        sem_destroy(&workSemaphore_);
    }

    void BundleAdjustment::startOptimizationWorker()
    {
        LOG(INFO, "Starting optimization worker thread");
        optimizationThread_ = std::thread(&BundleAdjustment::optimizationWorker, this);

        struct sched_param param
        {
            0
        };
        int result = pthread_setschedparam(optimizationThread_.native_handle(), SCHED_IDLE, &param);
        if (result != 0)
            LOG(WARN, "Failed to set SCHED_IDLE priority for optimization worker: " << result);
    }

    void BundleAdjustment::stopOptimizationWorker()
    {
        LOG(INFO, "Stopping optimization worker thread");
        shutdown_ = true;
        sem_post(&workSemaphore_);
        if (optimizationThread_.joinable())
        {
            optimizationThread_.join();
            LOG(INFO, "Optimization worker thread joined");
        }
    }

    void BundleAdjustment::accumulateSubmap(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                                            const std::shared_ptr<open3d::geometry::PointCloud> &cloud)
    {
        const double minDist = config_.bundle_adjustment.submap_min_distance;
        const double minAngle = config_.bundle_adjustment.submap_min_angle;

        if (lastAcceptedPose_.has_value())
        {
            const double dist = (pose->translation() - lastAcceptedPose_->translation()).norm();
            const double angle = lastAcceptedPose_->rotation().between(pose->rotation()).axisAngle().second;
            if (dist < minDist && angle < minAngle)
            {
                // debug-log the full BA state on every accumulation call
                LOG(DEBUG, "Submap rejected, BA state - " << pendingSubmaps_.size() << " active, "
                                                          << frozenSubmaps_.size() << " frozen");
                return;
            }
        }

        lastAcceptedPose_ = *pose;

        // undo SLAM world transform to store in body frame, then compute normals
        std::shared_ptr<open3d::geometry::PointCloud> pcdBody = std::make_shared<open3d::geometry::PointCloud>(*cloud);
        pcdBody->Transform(pose->inverse().matrix());
        pcdBody->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(7));

        PendingSubmap submap;
        submap.keyframeIdx = keyframeIdx;
        submap.pose = *pose;
        submap.pcd = std::move(pcdBody);

        LOG(INFO, "Accepted submap (BA #" << numAcceptedSubmaps << ") at keyframe " << keyframeIdx);
        LOG(DEBUG, "BA state - " << pendingSubmaps_.size() << " active, " << frozenSubmaps_.size() << " frozen");

        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            activeSubmapSnapshot_[keyframeIdx] = {submap.pose, submap.pcd};
        }
        incomingQueue_.push(std::move(submap));
        numAcceptedSubmaps++;
        sem_post(&workSemaphore_);
    }

    std::vector<std::shared_ptr<const FrozenSubmap>>
    BundleAdjustment::getGlobalMap(const gtsam::Pose3 &pose,
                                   const open3d::geometry::AxisAlignedBoundingBox &query_aabb_body, size_t N) const
    {
        std::vector<std::shared_ptr<const FrozenSubmap>> frozen;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            frozen = frozenSubmaps_;
        }

        if (frozen.empty())
            return {};

        const double margin = config_.bundle_adjustment.scan_to_map_registration.aabb_inflation_margin;

        // transform the body-frame query AABB into world frame via the 8 corners
        const Eigen::Vector3d &lo = query_aabb_body.min_bound_;
        const Eigen::Vector3d &hi = query_aabb_body.max_bound_;
        Eigen::Vector3d worldMin = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d worldMax = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
        // each corner is transformed to world frame, the component-wise min/max over all 8
        // gives the tightest axis-aligned bounds enclosing the rotated box
        for (int cx = 0; cx < 2; ++cx)
            for (int cy = 0; cy < 2; ++cy)
                for (int cz = 0; cz < 2; ++cz)
                {
                    Eigen::Vector3d corner(cx ? hi.x() : lo.x(), cy ? hi.y() : lo.y(), cz ? hi.z() : lo.z());
                    Eigen::Vector3d w = pose.transformFrom(corner);
                    worldMin = worldMin.cwiseMin(w);
                    worldMax = worldMax.cwiseMax(w);
                }
        // inflate by margin to tolerate drift between BA cycles
        worldMin.array() -= margin;
        worldMax.array() += margin;
        const open3d::geometry::AxisAlignedBoundingBox queryWorld(worldMin, worldMax);

        const Eigen::Vector3d queryCenter = pose.translation();

        struct Candidate
        {
            std::shared_ptr<const FrozenSubmap> submap;
            double distSq;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(frozen.size());

        for (const auto &sm : frozen)
        {
            // AABB overlap test: .array() makes comparisons component-wise, .any() fails on separation along any axis
            if ((queryWorld.min_bound_.array() > sm->aabb.max_bound_.array()).any())
                continue;
            if ((queryWorld.max_bound_.array() < sm->aabb.min_bound_.array()).any())
                continue;
            const double dSq = (queryCenter - sm->pose.translation()).squaredNorm();
            candidates.push_back({sm, dSq});
        }

        const size_t K = std::min(N, candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(K), candidates.end(),
                          [](const Candidate &a, const Candidate &b) { return a.distSq < b.distSq; });

        std::vector<std::shared_ptr<const FrozenSubmap>> result;
        result.reserve(K);
        for (size_t i = 0; i < K; ++i)
            result.push_back(std::move(candidates[i].submap));
        return result;
    }

    void BundleAdjustment::optimizationWorker(BundleAdjustment *self)
    {
        LOG(INFO, "Optimization worker thread started");

#ifdef __linux__
        pthread_setname_np(pthread_self(), "BA_Optimizer");
#endif

        while (!self->shutdown_)
        {
            // drain incoming queue into pending pool
            PendingSubmap incoming;
            while (self->incomingQueue_.try_pop(incoming))
                self->pendingSubmaps_.push_back(std::move(incoming));

            if (!self->pendingSubmaps_.empty())
            {
                // bootstrap: freeze the very first submap as the global anchor so subsequent
                // submaps always have at least one frozen reference to align against
                bool hasFrozen;
                {
                    std::lock_guard<std::mutex> lock(self->mapMutex_);
                    hasFrozen = !self->frozenSubmaps_.empty();
                }
                if (!hasFrozen)
                {
                    LOG(INFO, "Bootstrap: freezing submap " << self->pendingSubmaps_.front().keyframeIdx
                                                            << " as global-map anchor");
                    self->freezeSubmap(self->pendingSubmaps_.front());
                    self->pendingSubmaps_.erase(self->pendingSubmaps_.begin());
                }

                // guard for minimum number of active submaps in PGO to avoid adversarial optimization results
                if (self->pendingSubmaps_.size() >= MIN_ACTIVE_SUBMAPS)
                    self->backgroundArena_->execute([&] { self->optimizeGlobalMap(self->pendingSubmaps_); });
            }
            // let the optimization worker wait until new submaps are pending
            sem_wait(&self->workSemaphore_);
        }

        LOG(INFO, "Optimization worker thread shutting down");
    }

    void BundleAdjustment::optimizeGlobalMap(std::vector<PendingSubmap> &workingSet)
    {
        // cardinality of active nodes that can still be optimized
        const std::size_t A = workingSet.size();

        // sort by keyframe index so sequential edges follow insertion order
        std::sort(workingSet.begin(), workingSet.end(),
                  [](const PendingSubmap &a, const PendingSubmap &b) { return a.keyframeIdx < b.keyframeIdx; });

        std::vector<std::shared_ptr<const FrozenSubmap>> frozenSnap;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            frozenSnap = frozenSubmaps_;
        }
        // cardinality of frozen nodes which act purely as reference
        const std::size_t R = frozenSnap.size();

        // unified entry for the list containing both frozen and active submaps
        struct PoseGraphNode
        {
            uint32_t keyframeIdx;
            gtsam::Pose3 pose;
            bool isFrozen;
            std::size_t srcIdx; // index into workingSet (active) or frozenSnap (frozen)
            int pgoIdx;         // index in poseGraph.nodes_: [0,A) active, [A,A+R) frozen
        };

        std::vector<PoseGraphNode> poseGraphNodes;
        poseGraphNodes.reserve(A + R);
        for (std::size_t i = 0; i < A; ++i)
            poseGraphNodes.push_back({workingSet[i].keyframeIdx, workingSet[i].pose, false, i, static_cast<int>(i)});
        for (std::size_t i = 0; i < R; ++i)
            poseGraphNodes.push_back(
                {frozenSnap[i]->keyframeIdx, frozenSnap[i]->pose, true, i, static_cast<int>(A + i)});
        std::sort(poseGraphNodes.begin(), poseGraphNodes.end(),
                  [](const PoseGraphNode &a, const PoseGraphNode &b) { return a.keyframeIdx < b.keyframeIdx; });

        // helper fn to return the body-frame point cloud for a node in the graph
        auto getBodyPcd = [&](const PoseGraphNode &e) -> std::shared_ptr<const open3d::geometry::PointCloud>
        {
            if (!e.isFrozen)
                return workingSet[e.srcIdx].pcd;
            return frozenSnap[e.srcIdx]->pcdBody;
        };

        const open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria{
            1e-4, 1e-4, config_.bundle_adjustment.icp_iterations};
        const double icpMaxDist = config_.bundle_adjustment.icp_max_correspondence_distance;
        const double loopRadius = config_.bundle_adjustment.loop_closure_search_radius;
        const double minFitness = config_.bundle_adjustment.loop_closure_min_fitness;

        // phase 1: sequential ICP chain to refine active submap poses before PGO
        const std::size_t M = poseGraphNodes.size();
        for (std::size_t m = 0; m + 1 < M; ++m)
        {
            PoseGraphNode &ni = poseGraphNodes[m];
            PoseGraphNode &nj = poseGraphNodes[m + 1];
            if (nj.isFrozen || workingSet[nj.srcIdx].icpAligned)
                continue;
            const auto pcdI = getBodyPcd(ni);
            const auto pcdJ = getBodyPcd(nj);
            const Eigen::Matrix4d initialGuess = (ni.pose.inverse() * nj.pose).matrix();
            const auto result = open3d::pipelines::registration::RegistrationICP(
                *pcdJ, *pcdI, icpMaxDist, initialGuess,
                open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);
            if (result.fitness_ >= minFitness)
            {
                const gtsam::Pose3 refinedPose(ni.pose.matrix() * result.transformation_);
                nj.pose = refinedPose;
                workingSet[nj.srcIdx].pose = refinedPose;
                workingSet[nj.srcIdx].icpAligned = true;
                LOG(DEBUG,
                    "chain ICP kf" << ni.keyframeIdx << "->kf" << nj.keyframeIdx << " fitness=" << result.fitness_);
            }
            else
                LOG(DEBUG, "chain ICP kf" << ni.keyframeIdx << "->kf" << nj.keyframeIdx
                                          << " failed (fitness=" << result.fitness_ << ")");
        }

        // build pose graph: active nodes [0,A), frozen reference nodes [A,A+R)
        // nodes are added after phase 1 so they carry refined poses
        open3d::pipelines::registration::PoseGraph poseGraph;
        poseGraph.nodes_.reserve(A + R);
        for (std::size_t i = 0; i < A; ++i)
            poseGraph.nodes_.emplace_back(workingSet[i].pose.matrix());
        for (std::size_t i = 0; i < R; ++i)
            poseGraph.nodes_.emplace_back(frozenSnap[i]->pose.matrix());

        // fixed edges connect every frozen reference node to the origin frozenSnap[0] in a star topology
        // this holds the frozen subgraph rigid relative to the fixed reference node
        // the star uses R-1 edges instead of the R*(R-1)/2 of an all-pairs clique
        // NOTE that Open3Ds PGO currently does not support fixing variables,
        // so this is a workaround
        // - previously, added R^2 edges between all reference nodes
        // - the single loop replaces this with one edge from each node to the (truly fixed) origin node
        const Eigen::Matrix6d infoFixed = Eigen::Matrix6d::Identity() * FIXED_NODE_INFO_SCALE;
        const std::size_t edgesBeforeFrozenPass = poseGraph.edges_.size();
        for (std::size_t j = 1; j < R; ++j)
            poseGraph.edges_.emplace_back(static_cast<int>(A + j), static_cast<int>(A + 0),
                                          (frozenSnap[0]->pose.inverse() * frozenSnap[j]->pose).matrix(), infoFixed,
                                          /*uncertain=*/false);
        const std::size_t numFrozenEdges = poseGraph.edges_.size() - edgesBeforeFrozenPass;
        LOG(INFO, "frozen<->frozen pass: " << numFrozenEdges << " fixed edges added");

        // save pre-PGO poses after phase 1 so convergence deltas measure PGO-only correction
        std::vector<gtsam::Pose3> prevPoses(A);
        for (std::size_t i = 0; i < A; ++i)
            prevPoses[i] = workingSet[i].pose;
        const std::size_t edgesBeforeActivePass = poseGraph.edges_.size();
        for (std::size_t mi = 0; mi < M; ++mi)
        {
            constexpr std::size_t MAX_PGO_LOOP_CLOSURES = 3; // max. number of LCs when building pose graph
            std::size_t numLoopClosures = 0;                 // number of LCs made on a node

            for (std::size_t mj = mi + 1; mj < M; ++mj)
            {
                const PoseGraphNode &ni = poseGraphNodes[mi]; // earlier in keyframe order (source)
                const PoseGraphNode &nj = poseGraphNodes[mj]; // later in keyframe order (target)
                // don't insert "odometry" edges for frozen submaps,
                // this has already been done in the above nested loop
                if (ni.isFrozen && nj.isFrozen)
                    continue;
                const bool isSequential = (mj == mi + 1);
                if (!isSequential) // if nodes aren't odometry-adjacent ..
                {
                    // .. check whether they're close enough for loop closure tests
                    // TODO: we could pass this keyframe "covisibilty" info from the SLAM system to the BA!
                    if ((ni.pose.translation() - nj.pose.translation()).norm() > loopRadius)
                        continue;
                }
                const std::shared_ptr<const open3d::geometry::PointCloud> pcdJ = getBodyPcd(nj);
                const std::shared_ptr<const open3d::geometry::PointCloud> pcdI = getBodyPcd(ni);
                // sequential nodes get odometry edges from their latest poses,
                // ICP is only used to test for the information matrix of the relative pose
                if (isSequential)
                {
                    // odometry edge: Open3D convention is T = P_target^-1 * P_source
                    const Eigen::Matrix4d Tpgo = (nj.pose.inverse() * ni.pose).matrix();
                    const Eigen::Matrix6d infoMatrix =
                        open3d::pipelines::registration::GetInformationMatrixFromPointClouds(*pcdI, *pcdJ, icpMaxDist,
                                                                                             Tpgo);
                    poseGraph.edges_.emplace_back(ni.pgoIdx, nj.pgoIdx, Tpgo, infoMatrix, /*uncertain=*/false);
                    LOG(DEBUG, "seq edge kf" << ni.keyframeIdx << "->kf" << nj.keyframeIdx);
                }
                // non-sequential nodes get loop closure tested (already gated above)
                else
                {
                    // skip loop closure check if this node has already reached the max. number of loop closures
                    if (numLoopClosures >= MAX_PGO_LOOP_CLOSURES)
                        continue;
                    // loop closure: ICP with SLAM relative pose as initial guess
                    // Open3D edge (source=ni, target=nj) stores T = P_nj^-1 * P_ni,
                    // so ICP source is pcdI (ni) and target is pcdJ (nj)
                    const Eigen::Matrix4d initialGuess = (nj.pose.inverse() * ni.pose).matrix();
                    const auto icpResult = open3d::pipelines::registration::RegistrationICP(
                        *pcdI, *pcdJ, icpMaxDist, initialGuess,
                        open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);
                    // loop closure test from ICP result
                    if (icpResult.fitness_ < minFitness)
                        continue;
                    const Eigen::Matrix6d infoMatrix =
                        open3d::pipelines::registration::GetInformationMatrixFromPointClouds(*pcdI, *pcdJ, icpMaxDist,
                                                                                             icpResult.transformation_);
                    poseGraph.edges_.emplace_back(ni.pgoIdx, nj.pgoIdx, icpResult.transformation_, infoMatrix,
                                                  /*uncertain=*/true);
                    LOG(DEBUG, "loop edge kf" << ni.keyframeIdx << "->kf" << nj.keyframeIdx
                                              << " fitness=" << icpResult.fitness_);
                    // counter to bound max. number of loop closures per node
                    numLoopClosures++;
                }
            }
        }

        const std::size_t numActivePassEdges = poseGraph.edges_.size() - edgesBeforeActivePass;
        LOG(INFO, "active pass: " << numActivePassEdges << " sequential and loop closure edges added");

        LOG(INFO, "PGO graph: " << poseGraph.nodes_.size() << " nodes (" << A << " active + " << R << " ref), "
                                << poseGraph.edges_.size() << " edges");
        // run PGO
        if (!poseGraph.edges_.empty())
        {
            const int referenceNode = (R > 0) ? static_cast<int>(A) : 0;
            open3d::pipelines::registration::GlobalOptimization(
                poseGraph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
                open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(
                    /*max_iteration=*/20,
                    /*min_relative_increment=*/1e-4,
                    /*min_relative_residual_increment=*/1e-4,
                    /*min_right_term=*/1e-3,
                    /*min_residual=*/1e-4,
                    /*max_iteration_lm=*/6),
                open3d::pipelines::registration::GlobalOptimizationOption(
                    icpMaxDist, /*edge_prune_threshold=*/0.25, /*preference_loop_closure=*/1.0, referenceNode));
        }

        // read back optimized world poses and compute convergence deltas
        for (std::size_t i = 0; i < A; ++i)
        {
            const gtsam::Pose3 newPose(poseGraph.nodes_[i].pose_);
            workingSet[i].lastDeltaTranslation = (newPose.translation() - prevPoses[i].translation()).norm();
            const gtsam::Rot3 dR = prevPoses[i].rotation().between(newPose.rotation());
            workingSet[i].lastDeltaRotation = dR.axisAngle().second;
            workingSet[i].pose = newPose;
        }

        for (std::size_t i = 0; i < A; ++i)
            LOG(DEBUG, "PGO delta kf" << workingSet[i].keyframeIdx << ": dT=" << workingSet[i].lastDeltaTranslation
                                      << " dR=" << workingSet[i].lastDeltaRotation);

        // partition working set: freeze converged/capped, keep the rest in pendingSubmaps_
        const double convT = config_.bundle_adjustment.convergence_pose_delta_translation;
        const double convR = config_.bundle_adjustment.convergence_pose_delta_rotation;
        const uint32_t iterCap = static_cast<uint32_t>(config_.bundle_adjustment.max_align_iterations);

        std::vector<PendingSubmap> nextPending;
        nextPending.reserve(A);

        // no. of submaps that were frozen during one PGO pass
        std::size_t numSubmapsFrozen{0};

        for (std::size_t i = 0; i < A; ++i)
        {
            PendingSubmap &sub = workingSet[i];
            ++sub.alignIterations;
            const bool converged = sub.lastDeltaTranslation < convT && sub.lastDeltaRotation < convR;
            const bool capHit = sub.alignIterations >= iterCap;

            if (converged || capHit)
            {
                LOG(INFO, "Freezing submap " << sub.keyframeIdx << " (iters=" << sub.alignIterations
                                             << ", dT=" << sub.lastDeltaTranslation << ", dR=" << sub.lastDeltaRotation
                                             << ", capout=" << (capHit ? "yes" : "no") << ")");
                freezeSubmap(sub);
                numSubmapsFrozen++;
            }
            else
                nextPending.push_back(std::move(sub));
        }

        pendingSubmaps_ = std::move(nextPending);

        // update snapshot poses to reflect the result of this optimization cycle
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            for (const auto &sub : pendingSubmaps_)
            {
                auto it = activeSubmapSnapshot_.find(sub.keyframeIdx);
                if (it != activeSubmapSnapshot_.end())
                    it->second.pose = sub.pose;
            }
        }

        LOG(INFO, "Optimization cycle done - still active: " << pendingSubmaps_.size()
                                                             << ", froze: " << numSubmapsFrozen << " ("
                                                             << frozenSubmaps_.size() << " total)");
    }

    void BundleAdjustment::freezeSubmap(PendingSubmap &submap)
    {
        // apply final optimized pose to body-frame point cloud to produce the world-frame cloud
        std::shared_ptr<open3d::geometry::PointCloud> pcdWorld =
            std::make_shared<open3d::geometry::PointCloud>(*submap.pcd);
        pcdWorld->Transform(submap.pose.matrix());

        std::shared_ptr<FrozenSubmap> frozen = std::make_shared<FrozenSubmap>();
        frozen->keyframeIdx = submap.keyframeIdx;
        frozen->pose = submap.pose;
        frozen->pcdWorld = pcdWorld;
        frozen->pcdBody = submap.pcd;
        frozen->aabb = pcdWorld->GetAxisAlignedBoundingBox();

        std::lock_guard<std::mutex> lock(mapMutex_);
        activeSubmapSnapshot_.erase(submap.keyframeIdx);
        frozenSubmaps_.push_back(std::move(frozen));
        ++mapVersion_;
    }

    std::vector<std::shared_ptr<const FrozenSubmap>> BundleAdjustment::getAllFrozenSubmaps() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return frozenSubmaps_;
    }

    std::map<uint32_t, ActiveSubmap> BundleAdjustment::getAllActiveSubmaps() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return activeSubmapSnapshot_;
    }

    size_t BundleAdjustment::getNumFrozenSubmaps() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return frozenSubmaps_.size();
    }

    size_t BundleAdjustment::getNumActiveSubmaps() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return activeSubmapSnapshot_.size();
    }

} // namespace mapping
