#pragma once

#ifndef MAPPING_STATES_HPP_
#define MAPPING_STATES_HPP_

#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

#include <mapping/Config.hpp>
#include <mapping/types.hpp>

namespace mapping
{
    /// @ingroup states
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

        /// @brief Update a keyframe's submap to a new world pose
        /// @details Will internally compute the left-applied pose delta `w_dT_w * w_pts_l`.
        /// @param keyframeIdx Index of the keyframe to update
        /// @param w_T_l New pose in world frame
        void updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &w_T_l);

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
        mutable double tLastImu_{0}, tLastScan_{0};

        // setters for bottom-up accesses that should adhere to constness
        void setLastScanTime(double timestamp) const { tLastScan_ = timestamp; };
        void setLastImuTime(double timestamp) const { tLastImu_ = timestamp; };

        void setImuToLidarExtrinsic(const gtsam::Pose3 &newExtrinsic) { imu_T_lidar_ = newExtrinsic; };
        gtsam::Pose3 &getImuToLidarExtrinsic() const { return imu_T_lidar_; };

        std::map<uint32_t, double> &getKeyframeTimestamps() const { return keyframeTimestamps_; };
        const uint32_t &getLatestKeyframeIdx() const { return keyframeCounter_; };

        gtsam::Pose3 &lastKeyframePose() const { return *keyframePoses_.rbegin()->second; };
        const double &lastKeyframeTimestamp() const { return keyframeTimestamps_.rbegin()->second; };

        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &getKeyframeSubmaps() const { return keyframeSubmaps_; };
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> &getKeyframePoses() const { return keyframePoses_; };

        const gtsam::Values &getSmootherEstimate() const { return smootherEstimate_; };
        void setSmootherEstimate(const gtsam::Values &newEstimate) { smootherEstimate_ = newEstimate; };

        /// @brief Get the Markov blanket (MB) of a keyframe, i.e. all variables that are directly connected to it via a factor.
        /// WARNING: this is NOT exact because at the time of marginalization, we don't know if a new constraint will be added,
        /// but it will be very noisy anyway so it should be reasonably safe to ignore it.
        /// @param idxMarginalize Index of the keyframe to be marginalized, lower bound for MB.
        /// @param idxKeyframe Index of the current keyframe, upper bound for MB.
        gtsam::Values getMarkovBlanketForKeyframe(const uint32_t idxMarginalize, const uint32_t &idxKeyframe) const;

        /// -- archive & obtain marginalized submaps (for visualization purposes only) ---

        /// @brief Enable or disable collection of marginalized submaps.
        /// Disabled by default to avoid unbounded memory growth in headless mode.
        void setCollectMarginalizedSubmaps(bool enable)
        {
            collectMarginalizedSubmaps_ = enable;
        };

        /// @brief Return submaps that were marginalized since the last call and clear the buffer internally.
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> getMarginalizedSubmaps();


    private:
        /// @brief SYsetm lifecycle state
        SystemState systemState_ = SystemState::Initializing;
        /// @brief Counter for assigning unique indices to keyframes.
        uint32_t keyframeCounter_{0};
        // map data
        mutable std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps_;
        mutable std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> keyframePoses_;
        mutable std::map<uint32_t, double> keyframeTimestamps_;
        /// @brief whether to keep marginalized submap PCDs for visualization interfaces.
        bool collectMarginalizedSubmaps_ = false;
        /// @brief marginalized submap PCDs, cleared on each call to getMarginalizedSubmaps()
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> marginalizedSubmaps_;
        // estimator lifecycle values
        mutable gtsam::NavState w_X_curr_, w_X_preint_;
        gtsam::imuBias::ConstantBias currBias_;
        mutable gtsam::Pose3 imu_T_lidar_; // imu-lidar extrinsic calibration.
        /// @brief Latest estimate of the smoother after updates are completed.
        gtsam::Values smootherEstimate_;
    };
} // namespace mapping

#endif // MAPPING_STATES_HPP_