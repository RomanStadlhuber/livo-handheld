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
#include <cmath>
#include <pthread.h>
#include <semaphore.h>

SETUP_LOGS(DEBUG, "BundleAdjustment");

namespace mapping
{
    namespace
    {
        // stiffness applied to identity edges between fixed reference nodes to emulate
        // multi-fixed-node behavior that Open3D's PGO does not support natively
        constexpr double FIXED_NODE_INFO_SCALE = 1e8;

        // very small epsilon on rotation angle norm for numerical safety
        constexpr double ROTATION_EPS = 1e-12;

        // tiny union-find for the floating-component check
        struct UnionFind
        {
            std::vector<std::size_t> parent;
            explicit UnionFind(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), std::size_t{0}); }
            std::size_t find(std::size_t x)
            {
                while (parent[x] != x)
                {
                    parent[x] = parent[parent[x]];
                    x = parent[x];
                }
                return x;
            }
            void unite(std::size_t a, std::size_t b)
            {
                a = find(a);
                b = find(b);
                if (a != b)
                    parent[a] = b;
            }
        };

        Eigen::Vector3d centroidOf(const open3d::geometry::PointCloud &cloud)
        {
            return cloud.HasPoints() ? cloud.GetCenter() : Eigen::Vector3d::Zero();
        }
    } // namespace

    BundleAdjustment::BundleAdjustment(const MappingConfig &config) : config_(config), nextSegmentId_{1}
    {
        activeSegment_.id = 0;
        activeSegmentStartIdx_ = 0;
        activeSegment_.state = SegmentState::Accumulating;
        activeSegment_.pcdMerged = nullptr;

        sem_init(&workSemaphore_, 0, 0);

        LOG(INFO, "BundleAdjustment initialized with config");
    }

    BundleAdjustment::~BundleAdjustment()
    {
        if (optimizationThread_.joinable())
        {
            stopOptimizationWorker();
        }
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
        {
            LOG(WARN, "Failed to set SCHED_IDLE priority for optimization worker: " << result);
        }

#ifdef __linux__
        // pin the BA worker to the first half of the available cores
        // Open3D's TBB/OpenMP child threads inherit the affinity mask of their parent (main),
        // so this effectively caps the cores available to all ICP and normal-estimation operations,
        // leaving cores free for the tracking thread
        const int totalCpus = static_cast<int>(std::thread::hardware_concurrency()),
                  // number of physical CPU cores made available to the BA thread
            baCpus = std::max(1, totalCpus / 2);
        cpu_set_t cpuSet;  // "bitmask" indicating available CPUs
        CPU_ZERO(&cpuSet); // first, make all unavailable, then add-back the 1st half of physical cores
        for (int i = 0; i < baCpus; ++i)
            CPU_SET(i, &cpuSet);
        // CPU affinity is the only effective way to limit Open3D's resource usage here:
        // Open3D spawns TBB/OpenMP workers that inherit the affinity mask but not SCHED_IDLE,
        // so they run at full priority on whichever cores the mask permits
        if (pthread_setaffinity_np(optimizationThread_.native_handle(), sizeof(cpu_set_t), &cpuSet) != 0)
            LOG(WARN, "Failed to set CPU affinity for optimization worker");
        else
            LOG(INFO, "BA worker pinned to " << baCpus << " of " << totalCpus << " cores");
#endif
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

    void BundleAdjustment::accumulateSubmapToSegment(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                                                     const std::shared_ptr<open3d::geometry::PointCloud> &cloud)
    {
        const Eigen::Vector3d translation = pose->translation();

        // accumulate path length from every received pose, not just accepted ones, so the segment
        // sealing threshold reflects true travel distance even when submaps are skipped
        if (lastReceivedTranslation_.has_value())
            activeSegment_.accumulatedDistance += (translation - *lastReceivedTranslation_).norm();
        lastReceivedTranslation_ = translation;

        // helper: seal activeSegment_ and open a fresh one; the caller decides whether to then
        // insert the current submap into the new segment
        auto sealActiveSegment = [&](const char *reason)
        {
            LOG(INFO, "Segment " << activeSegment_.id << " sealed (" << reason
                                 << ", submaps=" << activeSegment_.submaps.size()
                                 << ", travel=" << activeSegment_.accumulatedDistance << "m)");
            activeSegment_.state = SegmentState::WaitingForRefinement;
            {
                std::lock_guard<std::mutex> lock(mapMutex_);
                for (const auto &[kfIdx, _] : activeSegment_.submaps)
                    sealedKeyframes_.push_back(kfIdx);
            }
            refinementQueue_.push(activeSegment_);
            sem_post(&workSemaphore_);

            GlobalMapSegment newSegment;
            newSegment.id = nextSegmentId_++;
            activeSegmentStartIdx_ = keyframeIdx;
            newSegment.state = SegmentState::Accumulating;
            newSegment.pcdMerged = nullptr;
            activeSegment_ = std::move(newSegment);
            LOG(INFO, "Created new active segment with id " << activeSegment_.id);
        };

        // gate: skip this submap if it is too close to the last accepted one
        const double minDist = config_.global_map_optimization.submap_min_distance;
        if (minDist > 0.0 && !activeSegment_.submaps.empty())
        {
            const double distFromLast =
                (translation - activeSegment_.submaps.rbegin()->second.first->translation()).norm();
            if (distFromLast < minDist)
            {
                // still check the distance threshold so a segment is not held open indefinitely
                // when the robot barely moves and every submap gets rejected
                if (activeSegment_.accumulatedDistance >= config_.global_map_optimization.segment_length)
                    sealActiveSegment("distance threshold");
                return;
            }
        }

        // seal early if the submap cap is about to be exceeded, then insert into the fresh segment
        const int maxSubmaps = config_.global_map_optimization.max_submaps_per_segment;
        if (maxSubmaps > 0 && static_cast<int>(activeSegment_.submaps.size()) >= maxSubmaps)
            sealActiveSegment("submap cap");

        activeSegment_.submaps[keyframeIdx] = {pose, cloud};
        LOG(DEBUG, "Accepted submap at keyframe " << keyframeIdx << " to segment " << activeSegment_.id
                                                  << " (n=" << activeSegment_.submaps.size()
                                                  << ", travel=" << activeSegment_.accumulatedDistance << "m)");

        if (activeSegment_.accumulatedDistance >= config_.global_map_optimization.segment_length)
            sealActiveSegment("distance threshold");
    }

    std::shared_ptr<const FrozenMapSnapshot> BundleAdjustment::getGlobalMap(const gtsam::Pose3 &pose,
                                                                            double radius) const
    {
        auto snapshot = std::make_shared<FrozenMapSnapshot>();
        const Eigen::Vector3d query = pose.translation();
        const double radiusSq = radius * radius;

        std::vector<std::shared_ptr<const FrozenSegment>> candidates;
        uint64_t version;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            candidates = frozenSegments_;
            version = mapVersion_;
        }

        snapshot->segments.reserve(candidates.size());
        for (const auto &seg : candidates)
        {
            if ((seg->centroid - query).squaredNorm() <= radiusSq)
                snapshot->segments.push_back(seg);
        }
        snapshot->version = version;

        return snapshot;
    }

    std::vector<std::size_t> BundleAdjustment::kNearestFrozen(const Eigen::Vector3d &centroid, std::size_t k) const
    {
        // worker thread is the sole writer of frozenSegments_, so a snapshot is fine
        std::vector<std::shared_ptr<const FrozenSegment>> snapshot;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            snapshot = frozenSegments_;
        }

        std::vector<std::pair<double, std::size_t>> scored;
        scored.reserve(snapshot.size());
        for (std::size_t i = 0; i < snapshot.size(); ++i)
            scored.emplace_back((snapshot[i]->centroid - centroid).squaredNorm(), i);

        const std::size_t take = std::min(k, scored.size());
        std::partial_sort(scored.begin(), scored.begin() + take, scored.end(),
                          [](const auto &a, const auto &b) { return a.first < b.first; });

        std::vector<std::size_t> result;
        result.reserve(take);
        for (std::size_t i = 0; i < take; ++i)
            result.push_back(scored[i].second);
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
            // 1. drain refinement queue: the very first refined segment becomes the frozen anchor
            //    immediately; all subsequent ones are queued for inter-segment alignment
            GlobalMapSegment segment;
            while (self->refinementQueue_.try_pop(segment))
            {
                LOG(DEBUG, "Refining segment " << segment.id);
                self->refineSegment(segment);

                bool hasFrozen;
                {
                    std::lock_guard<std::mutex> lock(self->mapMutex_);
                    hasFrozen = !self->frozenSegments_.empty();
                }
                if (!hasFrozen)
                {
                    LOG(INFO, "Bootstrap: freezing segment " << segment.id << " as global-map anchor");
                    self->freezeSegment(segment);
                }
                else
                {
                    segment.state = SegmentState::WaitingForAlignment;
                    self->alignmentQueue_.push(std::move(segment));
                }
            }

            // 2. drain alignment queue into a fresh batch
            std::vector<GlobalMapSegment> newBatch;
            while (self->alignmentQueue_.try_pop(segment))
                newBatch.push_back(std::move(segment));

            // 3. if there is a new batch, run unified PGO over pendingSegments_ combined with newBatch
            if (!newBatch.empty())
            {
                std::vector<GlobalMapSegment> workingSet = std::move(self->pendingSegments_);
                self->pendingSegments_.clear();
                workingSet.reserve(workingSet.size() + newBatch.size());
                for (auto &seg : newBatch)
                    workingSet.push_back(std::move(seg));

                LOG(INFO, "Running alignment on " << workingSet.size() << " segments ("
                                                  << "pending was " << workingSet.size() - newBatch.size()
                                                  << ", new batch " << newBatch.size() << ")");
                self->alignAllSegments(workingSet);
            }

            // 4. block until more work or shutdown
            sem_wait(&self->workSemaphore_);
        }

        LOG(INFO, "Optimization worker thread shutting down");
    }

    void BundleAdjustment::refineSegment(GlobalMapSegment &segment)
    {
        const std::size_t N = segment.submaps.size();
        LOG(INFO, "Refining segment " << segment.id << " with " << N << " submaps");

        segment.pcdMerged = std::make_shared<open3d::geometry::PointCloud>();

        if (N < 2)
        {
            for (auto &[kfIdx, poseAndCloud] : segment.submaps)
                *segment.pcdMerged += *poseAndCloud.second;
            return;
        }

        struct SubmapEntry
        {
            open3d::geometry::PointCloud *cloud;
            gtsam::Pose3 odomPose;
        };
        std::vector<SubmapEntry> ordered;
        ordered.reserve(N);

        // transform all submaps to their local LiDAR frame and estimate normals for point-to-plane ICP;
        // KNN search is significantly faster than the default radius search on dense clouds
        for (auto &[kfIdx, poseAndCloud] : segment.submaps)
        {
            auto &[pose, cloud] = poseAndCloud;
            cloud->Transform(pose->inverse().matrix());
            cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(7));
            ordered.push_back({cloud.get(), *pose});
        }

        open3d::pipelines::registration::PoseGraph poseGraph;
        poseGraph.nodes_.reserve(N);
        poseGraph.edges_.reserve(N * N);

        std::vector<gtsam::Pose3> updatedPoses;
        updatedPoses.reserve(N);

        poseGraph.nodes_.emplace_back(ordered[0].odomPose.matrix());
        updatedPoses.push_back(ordered[0].odomPose);

        // pass 1: sequential odometry edges
        // use a fixed information matrix for odometry edges: they are marked uncertain=false so the
        // optimizer treats them as hard constraints regardless of the exact per-element weights,
        // and skipping GetInformationMatrixFromPointClouds saves N-1 ICP calls per segment
        const Eigen::Matrix6d odomInfo = Eigen::Matrix6d::Identity() * 100.0;
        for (std::size_t i = 0; i < N - 1; ++i)
        {
            const std::size_t j = i + 1;
            const gtsam::Pose3 odomDelta = ordered[i].odomPose.between(ordered[j].odomPose);

            updatedPoses.push_back(updatedPoses[i].compose(odomDelta));
            poseGraph.nodes_.emplace_back(updatedPoses[j].matrix());

            poseGraph.edges_.emplace_back(static_cast<int>(j), static_cast<int>(i), odomDelta.matrix(), odomInfo,
                                          /*uncertain=*/false);

            LOG(DEBUG, "odometry edge " << i << "->" << j);
        }

        // pass 2: loop closure edges between nearby non-consecutive nodes
        const open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria{
            1e-4, 1e-4, config_.global_map_optimization.icp_iterations};
        const double loopClosureMaxDist = config_.global_map_optimization.loop_closure_max_distance,
                     loopClosureMaxAngle = config_.global_map_optimization.loop_closure_max_angle,
                     loopClosureMinFitness = config_.global_map_optimization.loop_closure_min_fitness;

        for (std::size_t i = 0; i < N; ++i)
        {
            for (std::size_t j = i + 2; j < N; ++j)
            {
                const double dist = (updatedPoses[i].translation() - updatedPoses[j].translation()).norm();
                if (dist > loopClosureMaxDist)
                    continue;

                const double angle = updatedPoses[i].rotation().between(updatedPoses[j].rotation()).axisAngle().second;
                if (angle > loopClosureMaxAngle)
                    continue;

                const gtsam::Pose3 i_T_j_init = updatedPoses[i].between(updatedPoses[j]);

                const auto icpResult = open3d::pipelines::registration::RegistrationICP(
                    *ordered[j].cloud, *ordered[i].cloud,
                    config_.global_map_optimization.icp_max_correspondence_distance, i_T_j_init.matrix(),
                    open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);

                if (icpResult.fitness_ < loopClosureMinFitness)
                    continue;

                const auto infoMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                    *ordered[j].cloud, *ordered[i].cloud,
                    config_.global_map_optimization.icp_max_correspondence_distance, icpResult.transformation_);

                poseGraph.edges_.emplace_back(static_cast<int>(j), static_cast<int>(i), icpResult.transformation_,
                                              infoMatrix, /*uncertain=*/true);

                LOG(DEBUG,
                    "loop closure edge " << i << "<->" << j << " dist=" << dist << " fitness=" << icpResult.fitness_);
            }
        }

        LOG(INFO, "Segment " << segment.id << " pose graph: " << poseGraph.nodes_.size() << " nodes, "
                             << poseGraph.edges_.size() << " edges");

        // pass 3: global pose graph optimization
        open3d::pipelines::registration::GlobalOptimization(
            poseGraph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
            open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(),
            open3d::pipelines::registration::GlobalOptimizationOption(
                config_.global_map_optimization.icp_max_correspondence_distance,
                /*edge_prune_threshold=*/0.25,
                /*preference_loop_closure=*/1.0,
                /*reference_node=*/0));

        // pass 4: apply optimized poses and merge into pcdMerged
        for (std::size_t k = 0; k < N; ++k)
        {
            ordered[k].cloud->Transform(poseGraph.nodes_[k].pose_);
            *segment.pcdMerged += *ordered[k].cloud;
        }

        segment.pcdMerged = segment.pcdMerged->VoxelDownSample(config_.global_map_optimization.refinement_voxel_size);
        LOG(INFO, "Segment " << segment.id << " refined: " << segment.pcdMerged->points_.size() << " points");
    }

    void BundleAdjustment::alignAllSegments(std::vector<GlobalMapSegment> &workingSet)
    {
        const std::size_t M = workingSet.size();
        if (M == 0)
            return;

        // resolve k-nearest frozen references for each free segment
        std::vector<Eigen::Vector3d> freeCentroids;
        freeCentroids.reserve(M);
        for (const auto &seg : workingSet)
            freeCentroids.push_back(centroidOf(*seg.pcdMerged));

        const std::size_t k = static_cast<std::size_t>(config_.global_map_optimization.k_nearest_frozen);

        std::vector<std::shared_ptr<const FrozenSegment>> frozenSnap;
        {
            std::lock_guard<std::mutex> lock(mapMutex_);
            frozenSnap = frozenSegments_;
        }

        // collect union of selected reference indices and per-free reference lists
        std::vector<std::vector<std::size_t>> perFreeRefs(M);
        std::vector<std::size_t> refUnionSorted;
        if (!frozenSnap.empty())
        {
            std::vector<bool> picked(frozenSnap.size(), false);
            for (std::size_t n = 0; n < M; ++n)
            {
                std::vector<std::pair<double, std::size_t>> scored;
                scored.reserve(frozenSnap.size());
                for (std::size_t i = 0; i < frozenSnap.size(); ++i)
                    scored.emplace_back((frozenSnap[i]->centroid - freeCentroids[n]).squaredNorm(), i);
                const std::size_t take = std::min(k, scored.size());
                std::partial_sort(scored.begin(), scored.begin() + take, scored.end(),
                                  [](const auto &a, const auto &b) { return a.first < b.first; });
                perFreeRefs[n].reserve(take);
                for (std::size_t i = 0; i < take; ++i)
                {
                    const std::size_t refIdx = scored[i].second;
                    perFreeRefs[n].push_back(refIdx);
                    picked[refIdx] = true;
                }
            }
            for (std::size_t i = 0; i < frozenSnap.size(); ++i)
                if (picked[i])
                    refUnionSorted.push_back(i);
        }

        // map global frozen index -> PGO node index
        const std::size_t R = refUnionSorted.size();
        std::vector<std::size_t> frozenIdxToNode(frozenSnap.size(), 0);
        for (std::size_t i = 0; i < R; ++i)
            frozenIdxToNode[refUnionSorted[i]] = M + i;

        // build pose graph: free nodes [0,M), reference nodes [M, M+R), all initialized at identity
        // (clouds are in world frame, so any optimization correction emerges as the node pose itself)
        open3d::pipelines::registration::PoseGraph poseGraph;
        poseGraph.nodes_.reserve(M + R);
        poseGraph.edges_.reserve((M + R) * (M + R));
        for (std::size_t i = 0; i < M + R; ++i)
            poseGraph.nodes_.emplace_back(Eigen::Matrix4d::Identity());

        // reference-rigidity stiff edges: identity transforms with very high information.
        // emulates Open3D's missing "multi-fixed-node" feature; combined with reference_node=M
        // this pins all reference nodes in place.
        const Eigen::Matrix6d stiffInfo = Eigen::Matrix6d::Identity() * FIXED_NODE_INFO_SCALE;
        for (std::size_t i = 0; i + 1 < R; ++i)
        {
            for (std::size_t j = i + 1; j < R; ++j)
            {
                poseGraph.edges_.emplace_back(static_cast<int>(M + i), static_cast<int>(M + j),
                                              Eigen::Matrix4d::Identity(), stiffInfo, /*uncertain=*/false);
            }
        }

        const open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria{
            1e-4, 1e-4, config_.global_map_optimization.icp_iterations};
        const double segIcpMaxDist = config_.global_map_optimization.refinement_voxel_size * 2.0;
        const double segLoopMaxDist = config_.global_map_optimization.segment_loop_closure_max_distance;
        const double minFitness = config_.global_map_optimization.loop_closure_min_fitness;

        // free <-> free edges: sequential always added, non-sequential gated on centroid distance + fitness
        for (std::size_t i = 0; i < M; ++i)
        {
            for (std::size_t j = i + 1; j < M; ++j)
            {
                const bool isSequential = (j == i + 1);
                const double dist = (freeCentroids[i] - freeCentroids[j]).norm();

                if (!isSequential && dist > segLoopMaxDist)
                    continue;

                const auto icpResult = open3d::pipelines::registration::RegistrationICP(
                    *workingSet[j].pcdMerged, *workingSet[i].pcdMerged, segIcpMaxDist, Eigen::Matrix4d::Identity(),
                    open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);

                if (!isSequential && icpResult.fitness_ < minFitness)
                    continue;

                const auto infoMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                    *workingSet[j].pcdMerged, *workingSet[i].pcdMerged, segIcpMaxDist, icpResult.transformation_);

                poseGraph.edges_.emplace_back(static_cast<int>(j), static_cast<int>(i), icpResult.transformation_,
                                              infoMatrix, /*uncertain=*/!isSequential);

                LOG(DEBUG, (isSequential ? "sequential" : "loop closure") << " free edge " << i << "->" << j << " dist="
                                                                          << dist << " fitness=" << icpResult.fitness_);
            }
        }

        // free <-> reference edges: ICP between each free segment and its k-nearest frozen refs.
        // edges are buffered per ref node so we can promote the best-fitness candidate to
        // uncertain=false. open3d GlobalOptimization requires every node to be reachable through
        // certain edges; marking all free->ref edges as uncertain leaves each ref node isolated
        // in the certain-edge subgraph and the optimizer misbehaves. promoting the strongest
        // free->ref edge per ref node ensures the full graph (free nodes + ref nodes) is one
        // connected component in the certain-edge subgraph.
        struct FreeRefCandidate
        {
            int freeNode;
            int refNode;
            Eigen::Matrix4d transformation;
            Eigen::Matrix6d infoMatrix;
            double fitness;
        };
        std::vector<std::vector<FreeRefCandidate>> candidatesByRef(R);
        for (auto &v : candidatesByRef)
            v.reserve(M);

        for (std::size_t n = 0; n < M; ++n)
        {
            for (std::size_t refIdx : perFreeRefs[n])
            {
                const auto &refSeg = *frozenSnap[refIdx];
                const auto icpResult = open3d::pipelines::registration::RegistrationICP(
                    *workingSet[n].pcdMerged, *refSeg.legacyCloud, segIcpMaxDist, Eigen::Matrix4d::Identity(),
                    open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);

                if (icpResult.fitness_ < minFitness)
                {
                    LOG(WARN,
                        "free->ref loop closure failed (fitness " << icpResult.fitness_ << " < " << minFitness << ")");
                    continue;
                }

                const auto infoMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                    *workingSet[n].pcdMerged, *refSeg.legacyCloud, segIcpMaxDist, icpResult.transformation_);

                const std::size_t refNode = frozenIdxToNode[refIdx];
                candidatesByRef[refNode - M].push_back({static_cast<int>(n), static_cast<int>(refNode),
                                                        icpResult.transformation_, infoMatrix, icpResult.fitness_});

                LOG(DEBUG, "free->ref edge " << n << "->" << refNode << " (frozen id=" << refSeg.id
                                             << ") fitness=" << icpResult.fitness_);
            }
        }

        for (std::size_t r = 0; r < R; ++r)
        {
            auto &candidates = candidatesByRef[r];
            if (candidates.empty())
                continue;
            const auto bestIt = std::max_element(candidates.begin(), candidates.end(),
                                                 [](const FreeRefCandidate &a, const FreeRefCandidate &b)
                                                 { return a.fitness < b.fitness; });
            for (const auto &c : candidates)
            {
                const bool isBest = (&c == &*bestIt);
                poseGraph.edges_.emplace_back(c.freeNode, c.refNode, c.transformation, c.infoMatrix,
                                              /*uncertain=*/!isBest);
            }
        }

        LOG(INFO, "unified PGO: " << M << " free + " << R << " ref nodes, " << poseGraph.edges_.size() << " edges");

        // run global optimization only if there are constraints to satisfy
        const bool runOptimization = !poseGraph.edges_.empty();
        if (runOptimization)
        {
            // reference_node pins one node; with stiff identity edges between refs, the rest of
            // them stay rigidly attached. when there are no refs, pin node 0 by convention.
            const int referenceNode = (R > 0) ? static_cast<int>(M) : 0;
            open3d::pipelines::registration::GlobalOptimization(
                poseGraph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
                open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(),
                open3d::pipelines::registration::GlobalOptimizationOption(segIcpMaxDist,
                                                                          /*edge_prune_threshold=*/0.25,
                                                                          /*preference_loop_closure=*/1.0,
                                                                          referenceNode));
        }

        // apply per-free corrections, record delta magnitudes for convergence test
        for (std::size_t i = 0; i < M; ++i)
        {
            const Eigen::Matrix4d &delta = poseGraph.nodes_[i].pose_;
            workingSet[i].pcdMerged->Transform(delta);

            const Eigen::Matrix3d R_delta = delta.block<3, 3>(0, 0);
            const Eigen::Vector3d t_delta = delta.block<3, 1>(0, 3);
            const double rotAngle = Eigen::AngleAxisd(R_delta).angle();
            workingSet[i].lastDeltaTranslation = t_delta.norm();
            workingSet[i].lastDeltaRotation = std::abs(rotAngle) < ROTATION_EPS ? 0.0 : std::abs(rotAngle);
        }

        // floating check via union-find over all PGO nodes (only edges that were added count)
        UnionFind uf(M + R);
        for (const auto &edge : poseGraph.edges_)
            uf.unite(static_cast<std::size_t>(edge.source_node_id_), static_cast<std::size_t>(edge.target_node_id_));

        std::vector<bool> isFloating(M, true);
        if (R > 0)
        {
            const std::size_t refRoot = uf.find(M);
            for (std::size_t i = 0; i < M; ++i)
                isFloating[i] = (uf.find(i) != refRoot);
        }
        // when R == 0, every free segment is by definition floating

        // partition working set into frozen / pending
        std::vector<GlobalMapSegment> nextPending;
        nextPending.reserve(M);

        const double convT = config_.global_map_optimization.convergence_pose_delta_translation;
        const double convR = config_.global_map_optimization.convergence_pose_delta_rotation;
        const uint32_t iterCap = static_cast<uint32_t>(config_.global_map_optimization.max_align_iterations);

        for (std::size_t i = 0; i < M; ++i)
        {
            auto &seg = workingSet[i];

            if (isFloating[i])
            {
                // floating segments bypass the convergence/iteration check entirely: they sit in
                // pendingSegments_ until a future batch bridges them to a frozen segment
                LOG(DEBUG, "segment " << seg.id << " floating, returning to pending pool");
                nextPending.push_back(std::move(seg));
                continue;
            }

            ++seg.alignIterations;
            const bool converged = seg.lastDeltaTranslation < convT && seg.lastDeltaRotation < convR;
            const bool capHit = seg.alignIterations >= iterCap;

            if (converged || capHit)
            {
                LOG(INFO, "Freezing segment " << seg.id << " (iters=" << seg.alignIterations
                                              << ", dT=" << seg.lastDeltaTranslation << ", dR=" << seg.lastDeltaRotation
                                              << ", capHit=" << capHit << ")");
                freezeSegment(seg);
            }
            else
            {
                LOG(DEBUG, "segment " << seg.id << " not converged (dT=" << seg.lastDeltaTranslation
                                      << ", dR=" << seg.lastDeltaRotation << "), returning to pending pool");
                nextPending.push_back(std::move(seg));
            }
        }

        pendingSegments_ = std::move(nextPending);
        LOG(INFO, "Alignment cycle done: pending=" << pendingSegments_.size());
    }

    std::vector<std::shared_ptr<const FrozenSegment>> BundleAdjustment::getAllFrozenSegments() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return frozenSegments_;
    }

    void BundleAdjustment::freezeSegment(GlobalMapSegment &segment)
    {
        auto frozen = std::make_shared<FrozenSegment>();
        frozen->id = segment.id;
        frozen->legacyCloud = std::shared_ptr<const open3d::geometry::PointCloud>(segment.pcdMerged);
        frozen->centroid = centroidOf(*frozen->legacyCloud);
        frozen->aabb = frozen->legacyCloud->GetAxisAlignedBoundingBox();
        frozen->keyframeIndices.reserve(segment.submaps.size());
        for (const auto &[kfIdx, _] : segment.submaps)
            frozen->keyframeIndices.push_back(kfIdx);

        segment.state = SegmentState::Aligned;

        std::lock_guard<std::mutex> lock(mapMutex_);
        for (const auto &kfIdx : frozen->keyframeIndices)
            sealedKeyframes_.erase(std::remove(sealedKeyframes_.begin(), sealedKeyframes_.end(), kfIdx),
                                   sealedKeyframes_.end());
        frozenSegments_.push_back(std::move(frozen));
        ++mapVersion_;
    }

    std::vector<uint32_t> BundleAdjustment::getSealedKeyframeIndices() const
    {
        std::lock_guard<std::mutex> lock(mapMutex_);
        return sealedKeyframes_;
    }

} // namespace mapping
