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

#include <iostream>

namespace mapping
{
    /// @ingroup frontend_lidar
    class LidarFrontend
    {
    public:
        LidarFrontend() = default;
        ~LidarFrontend() = default;

        /// @brief Accumulated undistorted scans from the buffer into a single keyframe submap.
        /// **NOTE:** this will internally reset the undistorted scan buffer!
        /// @param states The current state of the system, used for reference (not modified).
        /// @param buffers The buffers to be used for accumulation.
        /// @return The accumulated submap PointCloud as shared_ptr.
        [[nodiscard]]
        std::shared_ptr<open3d::geometry::PointCloud> accumulateUndistortedScans(
            const States &states,
            Buffers &buffers,
            const MappingConfig &config);

        /// @brief Track the current keyframe submap against persistent "same plane point" clusters
        /// @param idxKeyframe Index of the current keyframe
        /// @param states Used to get the latest keyframe pose and submap.
        /// @param featureManager Used to access the clusters and their parameters.
        /// @param config Used to get the tracking parameters.
        /// @return True if the keyframe was tracked successfully
        bool trackScanPointsToClusters(
            const uint32_t &idxKeyframe,
            const States &states,
            FeatureManager &featureManager,
            const MappingConfig &config);
    };
} // namespace mapping

#endif // MAPPING_FRONTEND_LIDARFRONTEND_HPP_