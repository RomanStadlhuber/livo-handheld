/// @file
/// @ingroup global_map
#pragma once

#ifndef MAPPING_BUNDLEADJUSTMENT_HPP_
#define MAPPING_BUNDLEADJUSTMENT_HPP_

#include <mapping/types.hpp>
#include <mapping/Config.hpp>
#include <mapping/States.hpp>

#include <gtsam/geometry/Pose3.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/t/geometry/PointCloud.h>

#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <semaphore.h>

namespace mapping
{
    /// @ingroup global_map
    /// @brief Bundle adjustment module for managing global map optimization.
    /// @details Handles inter-segment and intra-segment pose graph optimization,
    /// accumulation of submaps into segments, and publishing of the optimized global map
    /// to the tracking thread via lock-free atomic snapshot reads.
    class BundleAdjustment
    {
    public:
        /// @brief Frozen snapshot of the global map for lock-free reads from tracking thread.
        struct FrozenMapSnapshot
        {
            /// @brief Point cloud in Open3D tensor format for efficient processing
            open3d::t::geometry::PointCloud cloud;
            /// @brief Number of aligned segments in this frozen map
            uint32_t alignedSegmentCount;
        };

        /// @brief Constructor initializing the bundle adjustment module with configuration.
        /// @param config Mapping system configuration
        explicit BundleAdjustment(const MappingConfig &config);

        /// @brief Destructor ensuring proper cleanup of optimization worker thread.
        ~BundleAdjustment();

        /// @brief Spawns the background optimization worker thread.
        /// @details The worker thread processes segments from the refinement and alignment
        /// queues, optimizes them, and updates the frozen global map snapshot.
        void startOptimizationWorker();

        /// @brief Gracefully stops and joins the optimization worker thread.
        /// @details Sets the shutdown flag and waits for the worker thread to complete.
        void stopOptimizationWorker();

        /// @brief Accumulate a submap into the currently active segment.
        /// @details Called from the tracking thread to add keyframe submaps to the active
        /// segment being accumulated. When the segment reaches its target size, it is moved
        /// to the refinement queue and a new segment becomes active.
        /// @param keyframeIdx Index of the keyframe for this submap
        /// @param pose Shared pointer to the LiDAR pose (world_T_lidar)
        /// @param cloud Shared pointer to the point cloud submap
        void accumulateSubmapToSegment(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                                       const std::shared_ptr<open3d::geometry::PointCloud> &cloud);

        /// @brief Get a lock-free read of the frozen global map snapshot.
        /// @details Returns the current frozen map snapshot, or nullptr if no map has been
        /// built yet. This is safe to call from the tracking thread without synchronization.
        /// @return Shared pointer to the frozen map snapshot, or nullptr
        std::shared_ptr<const FrozenMapSnapshot> getGlobalMap() const;

    private:
        /// @brief Static worker function for the optimization thread.
        /// @details Processes segments from queues, optimizes them, and updates the global map.
        /// @param self Pointer to the BundleAdjustment instance
        static void optimizationWorker(BundleAdjustment *self);

        /// @brief Perform intra-segment pose graph optimization.
        /// @details Optimizes the relative poses of submaps within a single segment
        /// using a local pose graph.
        /// @param segment The segment to refine
        void refineSegment(GlobalMapSegment &segment);

        /// @brief Perform inter-segment pose graph optimization.
        /// @details Optimizes the relative poses between all segments in the global map
        /// to ensure consistency across segment boundaries.
        /// @param segments All segments to align
        void alignAllSegments(const std::vector<GlobalMapSegment> &segments);

        /// @brief Build and publish the frozen map snapshot.
        /// @details Merges all aligned segments into a single frozen map snapshot in
        /// Open3D tensor format and publishes it via atomic pointer for tracking thread reads.
        void buildFrozenSnapshot();

        /// @brief Compute relative pose from one pose to another.
        /// @details Helper function to compute the transformation from poseA to poseB.
        /// @param poseA Reference pose
        /// @param poseB Target pose
        /// @return Relative pose: poseA^{-1} * poseB
        gtsam::Pose3 computeRelativePose(const gtsam::Pose3 &poseA, const gtsam::Pose3 &poseB) const;

        // Configuration
        /// @brief Mapping system configuration
        MappingConfig config_;

        // Threading and synchronization
        /// @brief Optimization worker thread
        std::thread optimizationThread_;
        /// @brief Flag to signal shutdown of optimization worker
        std::atomic<bool> shutdown_{false};

        // Work queues
        /// @brief Queue of segments waiting for intra-segment refinement
        SafeQueue<GlobalMapSegment> refinementQueue_;
        /// @brief Queue of segments waiting for inter-segment alignment
        SafeQueue<GlobalMapSegment> alignmentQueue_;
        /// @brief Semaphore signaled when work is pushed to refinementQueue_
        sem_t workSemaphore_;

        // Map state
        /// @brief All optimized segments from history
        std::vector<GlobalMapSegment> allSegments_;
        /// @brief Currently accumulating segment
        GlobalMapSegment activeSegment_;
        /// @brief Counter for assigning unique segment IDs
        uint32_t nextSegmentId_{0};

        // Global map snapshot
        /// @brief Frozen global map snapshot (protected by mutex for thread-safe updates)
        std::shared_ptr<const FrozenMapSnapshot> globalMapSnapshot_;
        /// @brief Mutex protecting access to globalMapSnapshot_
        mutable std::mutex snapshotMutex_;
    };
} // namespace mapping

#endif // MAPPING_BUNDLEADJUSTMENT_HPP_
