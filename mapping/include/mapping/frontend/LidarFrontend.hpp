/// @file
/// @ingroup frontend_lidar
#pragma once

#ifndef MAPPING_FRONTEND_LIDARFRONTEND_HPP_
#define MAPPING_FRONTEND_LIDARFRONTEND_HPP_

#include <mapping/types.hpp>
#include <mapping/Config.hpp>
#include <mapping/States.hpp>
#include <mapping/Buffers.hpp>
#include <mapping/backend/FeatureManager.hpp>

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>

namespace mapping
{
    /// @ingroup frontend_lidar
    class LidarFrontend
    {
    public:
        LidarFrontend() = default;
        ~LidarFrontend() = default;

        /// @brief Accumulated undistorted scans from the buffer into a single keyframe submap.
        /// @param states The current state of the system, used for reference (not modified).
        /// @param buffers The buffers to be used for accumulation.
        /// @return The accumulated submap PointCloud as shared_ptr.
        [[nodiscard]]
        std::shared_ptr<open3d::geometry::PointCloud> accumulateUndistortedScans(const States &states, Buffers &buffers,
                                                                                 const MappingConfig &config);

        /// @brief Track the current keyframe submap against persistent "same plane point" clusters
        /// @param idxKeyframe Index of the current keyframe
        /// @param states Used to get the latest keyframe pose and submap.
        /// @param featureManager Used to access the clusters and their parameters.
        /// @param config Used to get the tracking parameters.
        /// @return True if the keyframe was tracked successfully
        bool trackScanPointsToClusters(const uint32_t &idxKeyframe, const States &states,
                                       FeatureManager &featureManager, const MappingConfig &config);
    };

    /// @ingroup frontend_lidar
    /// @brief PD controller for adaptive voxel size to respond to scale of the scans environment.
    /// Implementation is based on [(Lee et. al, 2026)](https://arxiv.org/abs/2603.16273)
    ///
    /// **Note** that the controller is stateful in that it needs to keep track of the P-error
    /// in order to compute the D-error.
    /// @details The paper notes that due to the fact that the setpoint changes dynamically, the
    /// controllers integral term is omitted to avoid large jumps of the step response.
    class AdaptiveVoxelsizeController
    {
    public:
        AdaptiveVoxelsizeController() = default;
        ~AdaptiveVoxelsizeController() = default;

        /// @brief Use the current keyframe point cloud & timestamps to compute the
        /// controllers response at the current timestep.
        /// @returns The voxel-size as computed by the controller, or the default configured value
        /// when the controller is not active.
        [[nodiscard]]
        double update(const uint32_t idxKeyframe, States &states, MappingConfig &config);

        void reset(const double currentVoxelsize);

    private:
        /// @brief Internal sate management of the controller.
        enum ControllerState
        {
            /// @brief Controller not used, `update` will return default configured voxel size.
            inactive = 0,
            /// @brief Need to cold-start D-term first to avoid large jumps.
            /// Will already start computing in `update`, but will still return the configured voxel size.
            warmup = 1,
            /// @brief Controller ready for regular operation.
            active = 2
        };
        /// @brief adaptive voxel controller states
        /// **Note:** although the setpoint is a point count, for arithmetic it's beneficial
        /// keeping it a double for controller computation
        ControllerState state_;
        double currVoxelsize_{0.0}, setPointNError_{0.0}; // error must remain in state for D term
    };
} // namespace mapping

#endif // MAPPING_FRONTEND_LIDARFRONTEND_HPP_
