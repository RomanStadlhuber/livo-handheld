/// @file
/// @ingroup factors
#pragma once

#ifndef MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_
#define MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/JacobianFactorQ.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <memory>
#include <map>
#include <vector>

namespace mapping
{
    /// @ingroup factors
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
        /// @param planeCenter Plane center in the world frame
        /// @param noiseModel Point-to-plane noise model
        /// @param clusterId Identifier for the cluster associated with this factor, used for lookup to replace/delete.
        PointToPlaneFactor(
            const gtsam::KeyVector &keys,
            const gtsam::Pose3 &imu_T_lidar,
            const std::map<gtsam::Key, Eigen::Vector3d> &lidar_points,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel,
            const uint32_t &clusterId);

        /// @brief Extended constructor with optional calibration keys for temporal and extrinsic calibration.
        PointToPlaneFactor(
            const gtsam::KeyVector &poseKeys,
            const gtsam::Pose3 &imu_T_lidar,
            const std::map<gtsam::Key, Eigen::Vector3d> &lidar_points,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel,
            const uint32_t &clusterId,
            boost::optional<gtsam::Key> dtKey,
            boost::optional<gtsam::Key> extrinsicKey,
            const std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> &keyframeTwists = {});

        /// @brief Evaluate unwhitened error for all poses involved in this factor
        /// @param values Current estimates of all variables in the factor graph
        /// @param H Optional Jacobians w.r.t. each pose (one matrix per key)
        /// @return Error vector with one element per point measurement
        gtsam::Vector unwhitenedError(
            const gtsam::Values &values,
            boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override;


        /// @brief Linearize the point-to-plane residual into a JacobianFactor.
        ///
        /// This override is required because the residual `r` is already a mean squared
        /// point-to-plane distance (m²). If returned via `unwhitenedError`, GTSAM would
        /// square it again internally when forming the cost, yielding a quartic objective.
        /// Instead, we directly construct the whitened linear system:
        /// ```
        /// A = (1/sigma) * J,   b = -(1/sigma) * r
        /// ```
        /// and hand it to GTSAM as a JacobianFactor. GTSAM then solves the normal
        /// equations `A^T A dx = A^T b`, which is equivalent to Mahalanobis-norm
        /// minimization of `(1/2) * r^T * Sigma^{-1} * r` without any further squaring.
        /// @param x Current state estimate at which the factor is linearized.
        /// @return Whitened JacobianFactor representing the linearized system.
        boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values &x) const override;

        /// @brief Clone method required for GTSAM factor copying
        gtsam::NonlinearFactor::shared_ptr clone() const override;

        ///@brief Add a new keyframe to the cluster factor.
        void add(
            const gtsam::Key &key,
            const Eigen::Vector3d &lidar_point,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel);

        /// @brief Add a new keyframe with an associated twist for temporal calibration.
        void add(
            const gtsam::Key &key,
            const Eigen::Vector3d &lidar_point,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d> &twist);

        ///@brief Remove a keyframe from the cluster factor.
        void remove(
            const gtsam::Key &key,
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel);


        /// @brief Keys did not change, but new estiamtes lead to updated plane parameters.
        void updatePlaneParameters(
            const std::shared_ptr<Eigen::Vector3d> &planeNormal,
            const std::shared_ptr<Eigen::Vector3d> &planeCenter,
            const gtsam::SharedNoiseModel &noiseModel);

        /// @brief Diagnostic usage for evaluating error. Use for debugging.
        double evaluateErrorDiagnostic(const gtsam::Values &values) const;

        void print(const std::string &s = "", const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter) const override;

        // trigger, cannot (and should not) be undone
        void markInvalid() const { isInvalid_ = true; }

        /// @brief marginalize a key from the factor to create a new JacobianFactor constraint.
        /// Do this **before** removing the key associaton from the factor.
        /// @details **Note** that this is not the actual marginalization factor,
        /// as that will be created by the backend on its own. But it is meant to provide a prior
        /// given the fact that with the explicit marginalization where the latest key is removed,
        /// there would be no other constraint than the IMU preintegration which is too weak.
        gtsam::LinearContainerFactor::shared_ptr createMarginalizationFactor(
            const gtsam::Values& values,
            const gtsam::Key &keyToMarginalize
        ) const;

    private:
        std::pair<gtsam::Vector, std::vector<gtsam::Matrix>> computeErrorAndJacobians(const gtsam::Values &values) const;

        static gtsam::KeyVector buildFullKeys(
            const gtsam::KeyVector &poseKeys,
            boost::optional<gtsam::Key> dtKey,
            boost::optional<gtsam::Key> extrinsicKey);

    private:
        gtsam::Pose3 imu_T_lidar_;
        std::map<gtsam::Key, Eigen::Vector3d> lidar_points_;
        std::shared_ptr<Eigen::Vector3d> planeNormal_;
        std::shared_ptr<Eigen::Vector3d> planeCenter_;
        // safeguard flag to avoid optimizing invalid factors that are not yet removed
        mutable bool isInvalid_ = false;
        gtsam::SharedNoiseModel adaptiveNoiseModel_;

    public:
        /// @brief Identifier for the cluster associated with this factor used for lookup to replace/delete.
        const uint32_t clusterId_;

    private:
        size_t numPoseKeys_;                        // boundary: keys_[0..numPoseKeys_) are pose keys
        boost::optional<gtsam::Key> dtKey_;         // T(0) when temporal calibration enabled
        boost::optional<gtsam::Key> extrinsicKey_;  // E(0) when extrinsic calibration enabled
        /// @brief Per-keyframe precomputed twist for temporal extrapolation.
        /// twist = Logmap(delta_T_preintegration) / dt_since_last_keyframe.
        /// Stored as (angular_velocity, linear_velocity) in IMU frame.
        /// Only populated when temporal calibration is enabled.
        std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyframeTwists_;
    };

} // namespace mapping

#endif // MAPPING_FACTORS_POINTTOPLANEFACTOR_HPP_
