/// @file
/// @ingroup global_map
#pragma once

#ifndef MAPPING_BUNDLEADJUSTMENT_HPP_
#define MAPPING_BUNDLEADJUSTMENT_HPP_

#include <mapping/types.hpp>
#include <mapping/Config.hpp>

#include <gtsam/geometry/Pose3.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/GlobalOptimization.h>
#include <open3d/pipelines/registration/PoseGraph.h>

#include <tbb/task_arena.h>

#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>
#include <limits>
#include <semaphore.h>

namespace mapping
{
    /// @ingroup global_map
    /// @brief Single-phase global map optimization over individual submaps.
    /// @details Accepts submaps gated by distance and angle thresholds, maintains a pose
    /// graph of pending (non-frozen) submaps with k-nearest frozen submaps as fixed
    /// references, runs GlobalOptimization each cycle, and freezes submaps that converge
    /// or hit the iteration cap.
    class BundleAdjustment
    {
    public:
        explicit BundleAdjustment(const MappingConfig &config);
        ~BundleAdjustment();

        void startOptimizationWorker();
        void stopOptimizationWorker();

        /// @brief Offer a newly marginalized submap to the global optimizer.
        /// @details Accepted when travel since the last accepted pose exceeds
        /// submap_min_distance OR rotation exceeds submap_min_angle.
        void accumulateSubmap(uint32_t keyframeIdx, const std::shared_ptr<gtsam::Pose3> &pose,
                              const std::shared_ptr<open3d::geometry::PointCloud> &cloud);

        /// @brief Return a mutex-guarded copy of all frozen submaps.
        std::vector<std::shared_ptr<const FrozenSubmap>> getAllFrozenSubmaps() const;

        /// @brief Return keyframe indices of submaps that are pending optimization.
        /// @details These submaps are in an intermediate frame; callers should skip
        /// rendering them until they appear in frozenSubmaps_.
        std::vector<uint32_t> getPendingKeyframeIndices() const;

        /// @brief Query the frozen global map by pose proximity.
        /// @details Returns a snapshot of every frozen submap whose pose origin is within radius of the query pose.
        std::shared_ptr<const FrozenMapSnapshot> getGlobalMap(const gtsam::Pose3 &pose, double radius) const;

    private:
        /// @brief Per-submap state tracked internally during optimization cycles.
        struct PendingSubmap
        {
            uint32_t keyframeIdx;
            gtsam::Pose3 pose;                                 // world pose; updated each PGO cycle
            std::shared_ptr<open3d::geometry::PointCloud> pcd; // body-frame point cloud
            uint32_t alignIterations{0};
            double lastDeltaTranslation{std::numeric_limits<double>::infinity()};
            double lastDeltaRotation{std::numeric_limits<double>::infinity()};
        };

        static void optimizationWorker(BundleAdjustment *self);

        /// @brief Run one PGO cycle over workingSet against k-nearest frozen references.
        /// @details Partitions workingSet in place: converged/capped submaps are frozen;
        /// the remainder stays in pendingSubmaps_.
        void optimizeGlobalMap(std::vector<PendingSubmap> &workingSet);

        void freezeSubmap(PendingSubmap &submap);

        MappingConfig config_;

        std::thread optimizationThread_;
        std::atomic<bool> shutdown_{false};

        SafeQueue<PendingSubmap> incomingQueue_;
        sem_t workSemaphore_;

        std::vector<std::shared_ptr<const FrozenSubmap>> frozenSubmaps_;
        std::vector<PendingSubmap> pendingSubmaps_;
        uint64_t mapVersion_{0};

        /// @brief Keyframe indices currently in pendingSubmaps_; protected by mapMutex_.
        std::vector<uint32_t> pendingKeyframes_;
        mutable std::mutex mapMutex_;

        /// @brief Pose of the last accepted submap, used for the distance/angle gate.
        std::optional<gtsam::Pose3> lastAcceptedPose_;

        std::unique_ptr<tbb::task_arena> backgroundArena_;
    };
} // namespace mapping

#endif // MAPPING_BUNDLEADJUSTMENT_HPP_
