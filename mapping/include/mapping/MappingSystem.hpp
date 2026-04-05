/// @file
/// @ingroup system
#pragma once

#ifndef MAPPING_MAPPINGSYSTEM_HPP_
#define MAPPING_MAPPINGSYSTEM_HPP_

#include <mapping/types.hpp>
#include <mapping/Config.hpp>
#include <mapping/Buffers.hpp>
#include <mapping/States.hpp>
#include <mapping/frontend/ImuFrontend.hpp>
#include <mapping/frontend/LidarFrontend.hpp>
#include <mapping/frontend/CameraFrontend.hpp>
#include <mapping/frontend/RecoveryFrontend.hpp>
#include <mapping/backend/FeatureManager.hpp>
#include <mapping/backend/Smoother.hpp>

#include <gtsam/geometry/Pose3.h>

namespace mapping
{
    /// @ingroup system
    /// @brief LiDAR-Inertial Odometry and Mapping System
    /// @details Orchestrates IMU preintegration, LiDAR scan processing,
    /// keyframe management, and factor graph optimization using a fixed-lag smoother.
    /// NOTE: This class should not use ROS-specific code.
    class MappingSystem
    {
    public:
        explicit MappingSystem(const MappingConfig &config);
        MappingSystem();
        ~MappingSystem() = default;

        /// @brief Set the config after initialization.
        /// @details This should not be used to update the config during operation,
        /// but rather before starting to feed data.
        void setConfig(const MappingConfig &config);

        /// @brief Feed an IMU measurement to the system
        /// @param imu_data IMU acceleration and angular velocity
        /// @param timestamp Measurement timestamp
        void feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp);

        /// @brief Feed a LiDAR scan to the system
        /// @param lidar_data LiDAR points with per-point timestamps
        /// @param timestamp Scan timestamp
        void feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp);

        /// @brief Feed images to the system.
        /// @param cam_data The image (+ color space)
        /// @param timestamp Image timestamp
        void feedCamera(const std::shared_ptr<CameraData> &camera_data, double timestamp);

        /// @brief Process buffered measurements and update state estimate
        void update();

        /// @brief Get the timestamped states currently in the sliding window.
        SlidingWindowStates getStates() const;

        /// @brief Get the submap pointcloud of the latest keyframe.
        /// May return `nullptr` if no keyframes exist.
        std::shared_ptr<open3d::geometry::PointCloud> getCurrentSubmap() const;

        /// @brief Get a representation of the currently tracked clusters
        /// (for visualization and debugging purposes).
        std::map<ClusterId, PointCluster> getCurrentClusters() const;

        /// @brief Get the current keyframe counter value.
        uint32_t getKeyframeCount() const;

        /// @brief Drain and return submaps that were marginalized since the last call.
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> getMarginalizedSubmaps();

        /// @brief Enable or disable collection of marginalized submaps.
        /// Disabled by default to avoid unbounded memory growth in headless mode.
        void setCollectMarginalizedSubmaps(bool enable);

        /// @brief Get the current (possibly optimized) IMU-to-LiDAR extrinsic calibration.
        gtsam::Pose3 getImuToLidarExtrinsic() const { return states_.getImuToLidarExtrinsic(); }
        /// @brief Get the current (possibly optimized) temporal offset [s].
        double getTemporalOffset() const { return states_.getTemporalOffset(); }

    private:
        /// @brief Static state (assumption-based) system initialization.
        /// @details Uses all IMU measurements up to the first LiDAR timestamp
        /// to build a gravity-aligned initial pose and calibrate the IMU preintegrator.
        void initializeSystem();

        /// @brief Main tracking loop - preintegrate IMU, undistort scans,
        /// create keyframes, track clusters, and optimize the factor graph.
        void track();

        /// @brief Attempty to recover the full system state  using the `RecoveryFrontend`.
        ///
        /// If recovery is successful, it will re-initialize at the recovered state so that
        /// the system can keep tracking.
        void recoverState();

        /// @brief Remove old keyframes and their cluster associations outside the sliding window.
        /// @details Must be called BEFORE tracking so that marginalization factors
        /// are created before the smoother update.
        /// @param idxKeyframe Index of the current (newly created) keyframe.
        void marginalizeKeyframesOutsideSlidingWindow(const uint32_t &idxKeyframe);

        /// @brief Construct priors for initializing the Smoother.
        ///
        /// This will construct the part of the factor graph that fixes the prior state
        /// `X`, `V` and bias `B`.
        ///
        /// NOTE: in the case of system recovery, it still makes sense to use the last available bias estimate,
        /// while the pose and velocity will be recovered by the `RelocalizationFrontend`.
        ///
        /// USAGE: Use the returned `gtsam::NonlinearFactorGraph priors` with `smoother_.setPriors(idxKeyframe, priors, xPrior, bPrior);`.
        /// @param idxKeyframe Index of the initialization keyframe.
        /// @param xPrior Initial navigation state prior, obtained from static initialization or recovery.
        /// @param bPrior Initial bias estimate, obtained from initialization or,
        /// in case cf recovery, from the last available estimate.
        /// @return Factor graph that fixes the state prior.
        gtsam::NonlinearFactorGraph constructSystemPriors(
            const uint32_t &idxKeyframe,
            const gtsam::NavState &xPrior,
            const gtsam::imuBias::ConstantBias &bPrior) const;

        // subsystem instances
        Buffers buffers_;
        States states_;
        ImuFrontend imuFrontend_;
        LidarFrontend lidarFrontend_;
        CameraFrontend cameraFrontend_;
        FeatureManager featureManager_;
        Smoother smoother_;

        // configuration and cached extrinsic
        MappingConfig config_;
        gtsam::Pose3 imu_T_lidar_;
    };

} // namespace mapping

#endif // MAPPING_MAPPINGSYSTEM_HPP_
