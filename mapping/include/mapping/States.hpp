#pragma once

#ifndef MAPPING_STATES_HPP_
#define MAPPING_STATES_HPP_

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <mapping/types.hpp>

namespace mapping
{
    class States
    {
    public:
        States() = default;
        ~States() = default;

        /// @brief Get the current system lifecycle state.
        void setLifecycleState(SystemState newState) { systemState_ = newState; };
        /// @brief Get the current system lifecycle state.
        SystemState getLifecycleState() const { return systemState_; };

        /// @brief Store new keyframe submap and initialized pose
        /// @param keyframePose Pose of the keyframe in world frame
        /// @param keyframeTimestamp Timestamp of the keyframe
        /// @param ptrKeyframeSubmap Pointcloud of the keyframe submap
        /// @return Index of the newly created keyframe
        uint32_t createKeyframeSubmap(
            const gtsam::Pose3 &keyframePose,
            double keyframeTimestamp,
            std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap);

        /// @brief Remove a keyframe (submap, pose, timestamp).
        void removeKeyframe(const uint32_t idxKeyframe);

        /// @brief Get the current estimated state (propagated OR updated).
        gtsam::NavState getCurrentState() const { return w_X_curr_; };
        void setCurrentState(const gtsam::NavState &newState) { w_X_curr_ = newState; };

        /// @brief The reference state used for IMU preintegration.
        /// Should always be equal to the last state that preintegraiton was reset to,
        /// i.e. the last keyframe state.
        /// @details Bottom-up setters need to adhere to constness.
        gtsam::NavState getPreintegrationRefState() const { return w_X_preint_; };
        void setPreintegrationRefState(const gtsam::NavState &newState) const { w_X_preint_ = newState; };

        gtsam::imuBias::ConstantBias getCurrentBias() const { return currBias_; };
        void setCurrentBias(const gtsam::imuBias::ConstantBias &newBias) { currBias_ = newBias; };

        // system-relative timestamps of the last processed IMU or LiDAR measurement.
        mutable double tLastImu_, tLastScan_;

        // setters for bottom-up accesses that should adhere to constness
        void setLastScanTime(double timestamp) const { tLastScan_ = timestamp; };
        void setLastImuTime(double timestamp) const { tLastImu_ = timestamp; };

        gtsam::Pose3 &getImuToLidarExtrinsic() const { return imu_T_lidar_; };
        gtsam::Pose3 &lastKeyframePose() const { return *keyframePoses_.rbegin()->second; };
        const double &lastKeyframeTimestamp() const { return keyframeTimestamps_.rbegin()->second; };

        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &getKeyframeSubmaps() const { return keyframeSubmaps_; };

    private:
        /// @brief SYsetm lifecycle state
        SystemState systemState_;
        /// @brief Counter for assigning unique indices to keyframes.
        uint32_t keyframeCounter_;
        // map data
        mutable std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps_;
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> keyframePoses_;
        std::map<uint32_t, double> keyframeTimestamps_;
        // estimator lifecycle values
        mutable gtsam::NavState w_X_curr_, w_X_preint_;
        gtsam::imuBias::ConstantBias currBias_;
        mutable gtsam::Pose3 imu_T_lidar_; // imu-lidar extrinsic calibration.
    };
}

#endif // MAPPING_STATES_HPP_