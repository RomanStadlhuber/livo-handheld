#pragma once

#ifndef MAPPING_FACTORS_PLANARCLUSTERFACTOR_HPP_
#define MAPPING_FACTORS_PLANARCLUSTERFACTOR_HPP_

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <mapping/Config.hpp>

namespace mapping
{
    // Forward declaration of PlanarClusterFactor
    class PlanarClusterFactor : public gtsam::NoiseModelFactor
    {
    public:
        PlanarClusterFactor(
            const gtsam::KeyVector &keys,
            const gtsam::Pose3 &imu_T_lidar,
            const uint32_t &clusterId,
            const ClusteringConfig &clusteringConfig = ClusteringConfig());

        /// @brief Add LiDAR points associated with this cluster from a specific keyframe
        /// @param lidar_points  3D points associated with the cluster, in the LiDAR frame
        /// @param key Keyframe index (in keys vector) from which the points originate
        /// @param world_T_imu  Pose of the IMU in the world frame at the time of the keyframe
        void add(
            std::vector<Eigen::Vector3d &> &lidar_points,
            const gtsam::Key &key,
            const gtsam::Pose3 &world_T_imu);

        boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &x) const override;

    private:
        /// @brief Compute the error vector for the factor.
        /// Will internally re-compute the plane fit if necessary
        gtsam::Vector computeError(const gtsam::Values &x) const;
        /// @brief Compute the Jacobians for each keyframe pose in the factor.
        std::vector<gtsam::Matrix> computeJacobians(const gtsam::Values &x) const;
        /// @brief Evaluate the cluster's planarity and update plane parameters (modifies mutable cache)
        void evaluate(const std::map<gtsam::Key, gtsam::Pose3> &world_currentPoses_imu) const;
        /// @brief Check whether the plane model needs to be re-evaluated
        bool checkPlaneFit(const std::map<gtsam::Key, gtsam::Pose3> &world_currentPoses_imu) const;
        /// @brief Transform stored LiDAR points to world frame using the provided IMU pose (modifies mutable cache)
        void transformPointsToWorldFrame(const gtsam::Key &key, const gtsam::Pose3 &world_T_imu) const;
        /// @brief A shorthand for checking if the plane parameters have been yet evaluated.
        bool isEvaluated() const { return planeThickness_ >= 0.0; }
        /// @brief Key of the newest frame, which plane parameters should be evaluaded against.
        gtsam::Key newestKeyframe() const { return keys().back(); }

    private:
        /// @brief Extrinsic IMU-LiDAR calibration.
        const gtsam::Pose3 imu_T_lidar_;
        const uint32_t clusterId_;
        /// @brief LiDAR points in local frame (immutable after add())
        std::map<gtsam::Key, std::vector<Eigen::Vector3d &>> lidar_points_;
        /// @brief Number of points associated with this cluster across all keyframes.
        /// @details Is `double` to facilitate average computations.
        double numPoints_ = 0;
        const ClusteringConfig clusteringConfig_;

        // --- Mutable cached state (updated during const error evaluation) ---
        /// @brief Cached world-frame points (recomputed when poses change)
        mutable std::map<gtsam::Key, std::vector<Eigen::Vector3d>> world_points_;
        /// @brief IMU poses of the keyframes (in world frame) at time of last plane evaluation
        mutable std::map<gtsam::Key, gtsam::Pose3> world_poses_imu_;
        /// @brief Cached plane normal in world frame
        mutable Eigen::Vector3d clusterNormal_;
        /// @brief Cached plane centroid in world frame
        mutable Eigen::Vector3d clusterCenter_;
        /// @brief Cached plane thickness (mean squared distance to plane)
        mutable double planeThickness_ = -1.0;
        /// @brief Whether the current plane fit is valid
        mutable bool isValid_ = false;
        mutable gtsam::SharedNoiseModel adaptiveNoiseModel_;
        // thresholds for requiring a plane re-fit (based on frobenius norm of pose change)
        static constexpr double kThresholdPlaneFit = 1e-5;
    };

} // namespace mapping

#endif // MAPPING_FACTORS_PLANARCLUSTERFACTOR_HPP_