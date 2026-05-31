/// @file
/// @ingroup frontend_scan_to_map
#pragma once

#ifndef MAPPING_FRONTEND_SCANTOMAPFRONTEND_HPP_
#define MAPPING_FRONTEND_SCANTOMAPFRONTEND_HPP_

#include <mapping/types.hpp>
#include <mapping/Config.hpp>
#include <mapping/States.hpp>
#include <mapping/BundleAdjustment.hpp>

#include <open3d/geometry/PointCloud.h>
#include <open3d/t/geometry/PointCloud.h>
#include <open3d/t/pipelines/registration/Registration.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>

namespace mapping
{
    /// @ingroup frontend_scan_to_map
    /// @brief Scan-to-map registration frontend using the BA frozen global map.
    /// @details Handles both normal tracking registration (returns a pose prior factor)
    /// and state recovery (returns a full NavState with velocity).
    class ScanToMapFrontend
    {
    public:
        ScanToMapFrontend() = default;

        /// @param ba bundle adjustment instance owning the frozen global map
        /// @param config mapping system configuration
        ScanToMapFrontend(const BundleAdjustment &ba, const MappingConfig &config);

        /// @brief Register a scan against the frozen global map during normal tracking.
        /// @details Updates the registration cache, runs multi-scale ICP, and on success
        /// returns a pose prior factor for the smoother.
        /// @param pcdScan scan in LiDAR body frame
        /// @param world_T_imu predicted IMU pose in world frame
        /// @param states current system state, used to read the IMU-to-LiDAR extrinsic
        /// @param idxKeyframe keyframe index for the returned prior factor
        /// @return PriorFactor on X(idxKeyframe) if ICP fitness meets threshold, nullopt otherwise
        std::optional<gtsam::NonlinearFactor::shared_ptr> registerScanToMap(const open3d::geometry::PointCloud &pcdScan,
                                                                            const gtsam::Pose3 &world_T_imu,
                                                                            const States &states, uint32_t idxKeyframe);

        /// @brief Estimate the full navigation state for system recovery via scan-to-map ICP.
        /// @details On success, the velocity is recovered by rotating the predicted world-frame
        /// velocity from the predicted IMU orientation into the ICP-refined orientation.
        /// @param pcdScan scan in LiDAR body frame
        /// @param world_X_imu predicted IMU navigation state in world frame
        /// @param states current system state, used to read the IMU-to-LiDAR extrinsic
        /// @return recovered NavState with ICP-refined pose and rotated velocity, nullopt if no map available
        std::optional<gtsam::NavState> estimateRecoveryState(const open3d::geometry::PointCloud &pcdScan,
                                                             const gtsam::NavState &world_X_imu, const States &states);

    private:
        struct IcpOutcome
        {
            gtsam::Pose3 world_T_imu;
            open3d::t::geometry::PointCloud pcdScan; // tensor scan, needed for info matrix computation
            open3d::t::pipelines::registration::RegistrationResult result;
        };

        /// @brief Update the registration cache and run multi-scale ICP against the frozen global map.
        /// @details Returns nullopt with LOG(ERROR) if no frozen submaps exist or the AABB search
        /// yields no overlapping candidates. Fitness check is left to the caller.
        /// @param pcdScan scan in LiDAR body frame
        /// @param world_T_imu predicted IMU pose in world frame
        /// @param states current system state, used to read the IMU-to-LiDAR extrinsic
        std::optional<IcpOutcome> runScanToMapIcp(const open3d::geometry::PointCloud &pcdScan,
                                                  const gtsam::Pose3 &world_T_imu, const States &states);

        const BundleAdjustment *ba_{nullptr};
        MappingConfig config_;
        RegistrationCache registrationCache_;
    };

} // namespace mapping

#endif // MAPPING_FRONTEND_SCANTOMAPFRONTEND_HPP_
