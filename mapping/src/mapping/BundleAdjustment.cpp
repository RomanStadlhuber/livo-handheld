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

            // Check if segment has reached threshold (hardcoded 10.0 for now)
            if (distance >= 10.0)
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
        LOG(INFO, "Refining segment " << segment.id << " with " << segment.submaps.size() << " submaps");

        // For now: stub implementation that logs and merges point clouds
        // Actual ICP/pose graph optimization will be added in subsequent phases

        // Create merged point cloud by concatenating all submaps
        if (!segment.submaps.empty())
        {
            segment.pcdMerged = std::make_shared<open3d::geometry::PointCloud>();

            for (const auto &kv : segment.submaps)
            {
                const auto &submap = kv.second.second;
                if (submap && submap->HasPoints())
                {
                    *segment.pcdMerged += *submap;
                }
            }

            // Voxel downsample the merged cloud
            if (segment.pcdMerged->HasPoints())
            {
                segment.pcdMerged = segment.pcdMerged->VoxelDownSample(config_.lidar_frontend.voxel_size);

                LOG(DEBUG, "Segment " << segment.id << " merged: " << segment.pcdMerged->points_.size() << " points");
            }
        }
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
