/// @file
/// @ingroup bundle_adjustment
#include <mapping/BundleAdjustment.hpp>
#include <mapping/logging.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <pthread.h>
#include <semaphore.h>

SETUP_LOGS(DEBUG, "BundleAdjustment");

namespace mapping
{

    BundleAdjustment::BundleAdjustment(const MappingConfig &config) : config_(config), nextSegmentId_{1}
    {
        activeSegment_.id = 0;
        activeSegment_.startIdx = 0;
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

        // Spawn optimization thread
        optimizationThread_ = std::thread(&BundleAdjustment::optimizationWorker, this);

        // Set thread to SCHED_IDLE priority using pthread_setschedparam
        struct sched_param param
        {
            0
        };
        int result = pthread_setschedparam(optimizationThread_.native_handle(), SCHED_IDLE, &param);
        if (result != 0)
        {
            LOG(WARN, "Failed to set SCHED_IDLE priority for optimization worker: " << result);
        }
    }

    void BundleAdjustment::stopOptimizationWorker()
    {
        LOG(INFO, "Stopping optimization worker thread");

        shutdown_ = true;
        sem_post(&workSemaphore_);

        // Wait for worker thread to finish
        if (optimizationThread_.joinable())
        {
            optimizationThread_.join();
            LOG(INFO, "Optimization worker thread joined");
        }
    }

    void BundleAdjustment::accumulateSubmapToSegment(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                                                     const std::shared_ptr<open3d::geometry::PointCloud> &cloud)
    {
        // Add to active segment's submaps
        activeSegment_.submaps[keyframeIdx] = {pose, cloud};

        LOG(DEBUG, "Added submap at keyframe " << keyframeIdx << " to segment " << activeSegment_.id);

        // Compute distance from first submap to last submap in active segment
        if (!activeSegment_.submaps.empty())
        {
            const auto &firstSubmap = activeSegment_.submaps.at(activeSegment_.startIdx);
            const auto &lastSubmap = activeSegment_.submaps.rbegin()->second;

            const Eigen::Vector3d firstTranslation = firstSubmap.first->translation();
            const Eigen::Vector3d lastTranslation = lastSubmap.first->translation();
            double distance = (firstTranslation - lastTranslation).norm();

            LOG(DEBUG, "Segment " << activeSegment_.id << " accumulated distance: " << distance << "m");

            // Check if segment has reached threshold
            if (distance >= config_.global_map_optimization.segment_length)
            {
                LOG(INFO,
                    "Segment " << activeSegment_.id << " reached distance threshold. Moving to refinement queue.");

                // Set state to WaitingForRefinement
                activeSegment_.state = SegmentState::WaitingForRefinement;

                refinementQueue_.push(activeSegment_);
                sem_post(&workSemaphore_);

                // Create new active segment
                GlobalMapSegment newSegment;
                newSegment.id = nextSegmentId_++;
                newSegment.startIdx = keyframeIdx;
                newSegment.state = SegmentState::Accumulating;
                newSegment.pcdMerged = nullptr;

                activeSegment_ = newSegment;

                LOG(INFO, "Created new active segment with id " << activeSegment_.id);
            }
        }
    }

    std::shared_ptr<const BundleAdjustment::FrozenMapSnapshot> BundleAdjustment::getGlobalMap() const
    {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        return globalMapSnapshot_;
    }

    void BundleAdjustment::optimizationWorker(BundleAdjustment *self)
    {
        LOG(INFO, "Optimization worker thread started");

        // Set thread name for debugging (optional, Linux-only)
#ifdef __linux__
        pthread_setname_np(pthread_self(), "BA_Optimizer");
#endif

        // Main worker loop
        while (!self->shutdown_)
        {
            // 1. Drain refinement queue
            GlobalMapSegment segment;
            while (self->refinementQueue_.try_pop(segment))
            {
                LOG(DEBUG, "Refining segment " << segment.id);
                self->refineSegment(segment);

                segment.state = SegmentState::WaitingForAlignment;
                self->alignmentQueue_.push(std::move(segment));

                LOG(DEBUG, "Segment " << segment.id << " moved to alignment queue");
            }

            // 2. Drain alignment queue
            std::vector<GlobalMapSegment> alignedSegments;
            while (self->alignmentQueue_.try_pop(segment))
            {
                alignedSegments.push_back(std::move(segment));
            }

            // 3. If we have aligned segments, align them and build snapshot
            if (!alignedSegments.empty())
            {
                LOG(INFO, "Aligning " << alignedSegments.size() << " segments");
                self->alignAllSegments(alignedSegments);

                // Mark all segments as aligned and add to allSegments
                for (auto &seg : alignedSegments)
                {
                    seg.state = SegmentState::Aligned;
                    self->allSegments_.push_back(std::move(seg));
                }

                LOG(INFO, "Total aligned segments: " << self->allSegments_.size());

                // Build and publish frozen snapshot
                self->buildFrozenSnapshot();
            }

            // 4. Wait for new work
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

        // ordered submap entries for random access in the loop closure pass
        struct SubmapEntry
        {
            open3d::geometry::PointCloud *cloud;
            gtsam::Pose3 odomPose;
        };
        std::vector<SubmapEntry> ordered;
        ordered.reserve(N);

        // transform all submaps to their local LiDAR frame and estimate normals for point-to-plane ICP
        for (auto &[kfIdx, poseAndCloud] : segment.submaps)
        {
            auto &[pose, cloud] = poseAndCloud;
            cloud->Transform(pose->inverse().matrix());
            cloud->EstimateNormals();
            ordered.push_back({cloud.get(), *pose});
        }

        open3d::pipelines::registration::PoseGraph poseGraph;
        poseGraph.nodes_.reserve(N);
        poseGraph.edges_.reserve(N - 1);

        // updated world poses chained from ICP results, used to init loop closure ICP
        std::vector<gtsam::Pose3> updatedPoses;
        updatedPoses.reserve(N);

        // first node is the anchor — fixed at its odometry pose
        poseGraph.nodes_.emplace_back(ordered[0].odomPose.matrix());
        updatedPoses.push_back(ordered[0].odomPose);

        // --- pass 1: sequential odometry edges ---
        for (std::size_t i = 0; i < N - 1; ++i)
        {
            const std::size_t j = i + 1;
            const gtsam::Pose3 odomDelta = ordered[i].odomPose.between(ordered[j].odomPose);

            /* NOTE: skip ICP for odometry edges: drift over short segment lengths will be negligible
             * given well-initialized LiDAR-inertial odometry, so the odometry delta is used directly.
             * The information matrix is still estimated from the geometry at the odometry alignment.
            const auto icpResult = open3d::pipelines::registration::RegistrationICP(
                *ordered[j].cloud, *ordered[i].cloud,
                config_.global_map_optimization.icp_max_correspondence_distance, odomDelta.matrix(),
                open3d::pipelines::registration::TransformationEstimationPointToPlane(), icpCriteria);
            */

            const auto infoMatrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
                *ordered[j].cloud, *ordered[i].cloud, config_.global_map_optimization.icp_max_correspondence_distance,
                odomDelta.matrix());

            // chain updated world pose: world_T_j = world_T_i * i_T_j_odom
            updatedPoses.push_back(updatedPoses[i].compose(odomDelta));
            poseGraph.nodes_.emplace_back(updatedPoses[j].matrix());

            // edge: source=j, target=i, transformation=i_T_j (target_T_source convention)
            poseGraph.edges_.emplace_back(static_cast<int>(j), static_cast<int>(i), odomDelta.matrix(), infoMatrix,
                                          /*uncertain=*/false);

            LOG(DEBUG, "odometry edge " << i << "->" << j);
        }

        // --- pass 2: loop closure edges between nearby non-consecutive nodes ---
        const open3d::pipelines::registration::ICPConvergenceCriteria icpCriteria{
            1e-4, 1e-4, config_.global_map_optimization.icp_iterations};
        const double loopClosureMaxDist = config_.global_map_optimization.loop_closure_max_distance,
                     loopClosureMaxAngle = config_.global_map_optimization.loop_closure_max_angle,
                     loopClosureMinFitness = config_.global_map_optimization.loop_closure_min_fitness;

        // build the full pose graph with "loop closure" criteria between non-adjacent edges
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

        // --- pass 3: global pose graph optimization ---
        open3d::pipelines::registration::GlobalOptimization(
            poseGraph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
            open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(),
            open3d::pipelines::registration::GlobalOptimizationOption(
                config_.global_map_optimization.icp_max_correspondence_distance,
                // TODO: do these need to be configurable?
                /*edge_prune_threshold=*/0.25,
                /*preference_loop_closure=*/1.0,
                /*reference_node=*/0));

        // --- pass 4: apply optimized poses and merge into pcdMerged ---
        // submaps are still in their local LiDAR frames (transformed in the prep loop above)
        // transform each to world frame using the optimized node pose and accumulate
        for (std::size_t k = 0; k < N; ++k)
        {
            ordered[k].cloud->Transform(poseGraph.nodes_[k].pose_);
            *segment.pcdMerged += *ordered[k].cloud;
        }

        // TODO: improve PCD quality with outlier removal?
        segment.pcdMerged = segment.pcdMerged->VoxelDownSample(config_.global_map_optimization.refinement_voxel_size);
        LOG(INFO, "Segment " << segment.id << " refined: " << segment.pcdMerged->points_.size() << " points");
    }

    void BundleAdjustment::alignAllSegments(const std::vector<GlobalMapSegment> &segments)
    {
        LOG(INFO, "Aligning " << segments.size() << " segments");

        // For now: stub implementation that logs alignment
        // Actual global pose graph optimization will be added in subsequent phases
        // Currently, no pose updates are performed; segment poses remain unchanged
    }

    void BundleAdjustment::buildFrozenSnapshot()
    {
        LOG(INFO, "Building frozen map snapshot from " << allSegments_.size() << " aligned segments");

        auto snapshot = std::make_shared<FrozenMapSnapshot>();

        // Merge all aligned segments into a single legacy PointCloud first
        auto mergedLegacyCloud = std::make_shared<open3d::geometry::PointCloud>();

        for (const auto &segment : allSegments_)
        {
            if (segment.state == SegmentState::Aligned && segment.pcdMerged)
            {
                if (segment.pcdMerged->HasPoints())
                {
                    *mergedLegacyCloud += *segment.pcdMerged;
                }
            }
        }

        // Convert merged legacy point cloud to tensor format if it has points
        if (mergedLegacyCloud->HasPoints())
        {
            snapshot->cloud = open3d::t::geometry::PointCloud::FromLegacy(*mergedLegacyCloud);
        }
        else
        {
            snapshot->cloud = open3d::t::geometry::PointCloud();
        }

        snapshot->alignedSegmentCount = static_cast<uint32_t>(allSegments_.size());

        // Store snapshot (protected by mutex)
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            globalMapSnapshot_ = snapshot;
        }

        LOG(INFO, "Published frozen map snapshot with " << snapshot->alignedSegmentCount << " aligned segments and "
                                                        << (mergedLegacyCloud ? mergedLegacyCloud->points_.size() : 0)
                                                        << " points");
    }

    gtsam::Pose3 BundleAdjustment::computeRelativePose(const gtsam::Pose3 &poseA, const gtsam::Pose3 &poseB) const
    {
        return poseA.inverse().compose(poseB);
    }

} // namespace mapping
