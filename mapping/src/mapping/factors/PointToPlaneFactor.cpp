#include <mapping/factors/PointToPlaneFactor.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace mapping
{

    PointToPlaneFactor::PointToPlaneFactor(
        const gtsam::KeyVector &keys,
        const gtsam::Pose3 &imu_T_lidar,
        const std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> &scanPointsPerKey,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        double planeNormalOffsetD,
        const gtsam::SharedNoiseModel &noiseModel,
        const u_int32_t &clusterId)
        : NoiseModelFactor(noiseModel, keys),
          imu_T_lidar_(imu_T_lidar),
          scanPointsPerKey_(scanPointsPerKey),
          planeNormal_(planeNormal),
          planeNormalOffsetD_(planeNormalOffsetD),
          totalPoints_(0),
          clusterId_(clusterId)
    {
        // Calculate total number of points for dimensionality
        for (const auto &[keyIdx, points] : scanPointsPerKey_)
        {
            totalPoints_ += points.size();
        }
    }

    gtsam::Vector PointToPlaneFactor::unwhitenedError(
        const gtsam::Values &values,
        boost::optional<std::vector<gtsam::Matrix> &> H) const
    {
        gtsam::Vector errorVec(totalPoints_);

        // If Jacobians requested, initialize them
        if (H)
        {
            H->resize(keys().size());
            for (size_t i = 0; i < keys().size(); ++i)
            {
                (*H)[i] = gtsam::Matrix::Zero(totalPoints_, 6);
            }
        }

        size_t errorIdx = 0;

        // Iterate over each keyframe that has observations
        for (const auto &[keyIdx, scanPoints] : scanPointsPerKey_)
        {
            // Get the pose estimate for this keyframe
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(keys()[keyIdx]);
            const gtsam::Pose3 w_T_lidar = w_T_imu.compose(imu_T_lidar_);

            // Compute error and Jacobian for each point from this keyframe
            for (size_t ptIdx = 0; ptIdx < scanPoints.size(); ++ptIdx, ++errorIdx)
            {
                const gtsam::Point3 lidar_p(*scanPoints[ptIdx]);
                const gtsam::Point3 w_p = w_T_lidar.transformFrom(lidar_p);

                // Point-to-plane distance error
                errorVec(errorIdx) = planeNormal_->dot(w_p) - planeNormalOffsetD_;

                // Compute Jacobian if requested
                if (H)
                {
                    // Jacobian w.r.t. the pose of this keyframe
                    const Eigen::Matrix<double, 1, 3> nT = planeNormal_->transpose();

                    // Rotation component: -n^T * [w_p]_x (where [.]_x is skew-symmetric)
                    gtsam::Matrix16 D_error_D_pose;
                    D_error_D_pose.head<3>() = -nT * gtsam::skewSymmetric(w_p);

                    // Translation component: n^T * R
                    D_error_D_pose.tail<3>() = nT * w_T_imu.rotation().matrix();

                    // Assign to the appropriate Jacobian matrix
                    (*H)[keyIdx].row(errorIdx) = D_error_D_pose;
                }
            }
        }

        return errorVec;
    }

    gtsam::NonlinearFactor::shared_ptr PointToPlaneFactor::clone() const
    {
        return boost::make_shared<PointToPlaneFactor>(
            keys(), imu_T_lidar_, scanPointsPerKey_, planeNormal_, planeNormalOffsetD_, noiseModel(), clusterId_);
    }

    void PointToPlaneFactor::print(const std::string &s, const gtsam::KeyFormatter &keyFormatter) const
    {
        std::cout << s << "PointToPlaneFactor (cluster id:" << clusterId_ << ") connecting keys: ";
        for (const auto &key : keys())
        {
            std::cout << keyFormatter(key) << " ";
        }
        std::cout << "\n\tCluster Id: " << clusterId_ << "\n"
                  << "\tPlane normal: [" << planeNormal_->transpose() << "], d: " << planeNormalOffsetD_ << "\n"
                  << "\tTotal points: " << totalPoints_ << "\n"
                  << "\tNoise model: ";
        noiseModel()->print("");
    }

} // namespace mapping
