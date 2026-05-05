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
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/GlobalOptimization.h>
#include <open3d/pipelines/registration/PoseGraph.h>
#include <open3d/t/geometry/PointCloud.h>

#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>
#include <semaphore.h>

namespace mapping
{
    /// @ingroup global_map
    /// @brief Bundle adjustment module for managing global map optimization.
    /// @details Handles inter-segment and intra-segment pose graph optimization,
    /// accumulation of submaps into segments, and exposes a per-segment frozen map
    /// view via `getGlobalMap()` for the tracking thread.
    class BundleAdjustment
    {
    public:
        /// @brief Constructor initializing the bundle adjustment module with configuration.
        /// @param config Mapping system configuration
        explicit BundleAdjustment(const MappingConfig &config);

        /// @brief Destructor ensuring proper cleanup of optimization worker thread.
        ~BundleAdjustment();

        /// @brief Spawns the background optimization worker thread.
        void startOptimizationWorker();

        /// @brief Gracefully stops and joins the optimization worker thread.
        void stopOptimizationWorker();

        /// @brief Accumulate a submap into the currently active segment.
        /// @param keyframeIdx Index of the keyframe for this submap
        /// @param pose Shared pointer to the LiDAR pose (world_T_lidar)
        /// @param cloud Shared pointer to the point cloud submap
        void accumulateSubmapToSegment(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                                       const std::shared_ptr<open3d::geometry::PointCloud> &cloud);

        /// @brief Return a mutex-guarded copy of all frozen segments.
        /// @details Zero-copy of clouds — legacyCloud is already shared_ptr<const>.
        std::vector<std::shared_ptr<const FrozenSegment>> getAllFrozenSegments() const;

        /// @brief Return the set of keyframe indices that belong to sealed-but-not-yet-frozen segments.
        /// @details These submaps have been handed off to the refinement pipeline and their clouds
        /// may be temporarily in an intermediate frame; callers should skip rendering them until
        /// they appear in a frozen segment.
        std::vector<uint32_t> getSealedKeyframeIndices() const;

        /// @brief Query the frozen global map by pose proximity.
        /// @details Returns a snapshot containing every frozen segment whose centroid is
        /// within `radius` of the query translation. Tensor representations are not
        /// built here — consumers can trigger conversion lazily via `FrozenSegment::tensor()`.
        /// @param pose Query pose (only translation is used)
        /// @param radius Maximum centroid distance [m]
        /// @return Snapshot containing the matching frozen segments
        std::shared_ptr<const FrozenMapSnapshot> getGlobalMap(const gtsam::Pose3 &pose, double radius) const;

    private:
        /// @brief Worker thread entry point.
        static void optimizationWorker(BundleAdjustment *self);

        /// @brief Intra-segment pose graph optimization.
        void refineSegment(GlobalMapSegment &segment);

        /// @brief Unified inter-segment PGO over `pendingSegments_` union newBatch with k-nearest
        /// frozen segments as fixed references. Mutates the working set in place and partitions
        /// segments into frozen / pending after optimization.
        void alignAllSegments(std::vector<GlobalMapSegment> &workingSet);

        /// @brief Convert a converged segment to a FrozenSegment and append to frozenSegments_.
        void freezeSegment(GlobalMapSegment &segment);

        /// @brief Pick the k frozen segments closest to `centroid` by Euclidean distance.
        std::vector<std::size_t> kNearestFrozen(const Eigen::Vector3d &centroid, std::size_t k) const;

        // Configuration
        MappingConfig config_;

        // Threading
        std::thread optimizationThread_;
        std::atomic<bool> shutdown_{false};

        // Work queues
        SafeQueue<GlobalMapSegment> refinementQueue_;
        SafeQueue<GlobalMapSegment> alignmentQueue_;
        sem_t workSemaphore_;

        // Map state
        /// @brief Frozen, immutable segments (the "global map")
        std::vector<std::shared_ptr<const FrozenSegment>> frozenSegments_;
        /// @brief Segments under active alignment but not yet frozen (floating or unconverged)
        std::vector<GlobalMapSegment> pendingSegments_;
        /// @brief Currently accumulating segment (foreground writes from tracking thread)
        GlobalMapSegment activeSegment_;
        /// @brief Translation of the last submap received via accumulateSubmapToSegment,
        /// used to compute incremental path length independently of which submaps were accepted
        std::optional<Eigen::Vector3d> lastReceivedTranslation_;
        /// @brief Counter for assigning unique segment IDs
        uint32_t nextSegmentId_{0};
        /// @brief Monotonic version counter, bumped each time a segment freezes
        uint64_t mapVersion_{0};
        /// @brief Keyframe indices of segments that have been sealed but not yet frozen.
        /// Protected by mapMutex_.
        std::vector<uint32_t> sealedKeyframes_;
        /// @brief Mutex guarding `frozenSegments_`, `mapVersion_`, and `sealedKeyframes_`
        mutable std::mutex mapMutex_;
    };
} // namespace mapping

#endif // MAPPING_BUNDLEADJUSTMENT_HPP_
