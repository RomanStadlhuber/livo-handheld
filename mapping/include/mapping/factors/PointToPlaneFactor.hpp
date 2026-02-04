#pragma once

#ifndef MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_
#define MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace mapping
{

    /// @brief Point-to-plane factor supporting multiple keyframe poses
    /// @details This factor constrains multiple keyframe poses by enforcing that observed
    /// points from each keyframe lie on a common plane. The plane is defined by its normal
    /// and offset d such that n^T * x - d = 0 for all points on the plane.
    class PointToPlaneFactor : public gtsam::NoiseModelFactor
    {
    public:
        /// @brief Constructor for point-to-plane factor with multiple keyframe poses
        /// @param keys Vector of pose keys that this factor connects (e.g., X(0), X(1), X(3))
        /// @param imu_T_lidar Fixed extrinsic calibration from IMU to LiDAR frame
        /// @param scanPointsPerKey Map from key index (in keys vector) to scan points in that keyframe's LiDAR frame
        /// @param planeNormal Plane normal in the world frame
        /// @param planeNormalOffsetD Plane offset d in world frame (n^T * x - d = 0)
        /// @param noiseModel Point-to-plane noise model
        /// @param clusterId Identifier for the cluster associated with this factor, used for lookup to replace/delete.
        PointToPlaneFactor(
            const gtsam::KeyVector &keys,
            const gtsam::Pose3 &imu_T_lidar,
            const std::unordered_map<gtsam::Key, std::vector<std::shared_ptr<Eigen::Vector3d>>> &scanPointsPerKey,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            double planeNormalOffsetD,
            const gtsam::SharedNoiseModel &noiseModel,
            const uint32_t &clusterId);

        /// @brief Evaluate unwhitened error for all poses involved in this factor
        /// @param values Current estimates of all variables in the factor graph
        /// @param H Optional Jacobians w.r.t. each pose (one matrix per key)
        /// @return Error vector with one element per point measurement
        gtsam::Vector unwhitenedError(
            const gtsam::Values &values,
            boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override;

        boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &x) const override;

        /// @brief Clone method required for GTSAM factor copying
        gtsam::NonlinearFactor::shared_ptr clone() const override;

        /// @brief Get the total number of point measurements in this factor
        size_t numPoints() const { return totalPoints_; }

        void print(const std::string &s = "", const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override;

        // trigger, cannot (and should not) be undone
        void markInvalid() const { invalid = true; }

    private:
        std::pair<gtsam::Vector, std::vector<gtsam::Matrix>> computeErrorAndJacobians(const gtsam::Values &values) const;

    private:
        gtsam::Pose3 imu_T_lidar_;
        std::unordered_map<gtsam::Key, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey_;
        std::shared_ptr<Eigen::Vector3d> planeNormal_;
        double planeNormalOffsetD_;
        /// @brief Total number of points associated with this factor.
        size_t totalPoints_;
        // safeguard flag to avoid optimizing invalid factors that are not yet removed
        mutable bool invalid = false;

    public:
        /// @brief Identifier for the cluster associated with this factor used for lookup to replace/delete.
        const uint32_t clusterId_;
    };

} // namespace mapping

#endif // MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_