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
        const size_t numKeys = keys().size();

        // r is a scalar
        gtsam::Vector errorVec = gtsam::Vector::Zero(1);

        // H is a vector of jacobians for each variable
        // H_i is (1 x 6) for pose i
        if (H)
        {
            H->resize(numKeys);
            for (size_t i = 0; i < numKeys; ++i)
            {
                (*H)[i] = gtsam::Matrix::Zero(1, 6);
            }
        }

        // Iterate over each keyframe that has observations
        for (const auto &[keyIdx, scanPoints] : scanPointsPerKey_)
        {
            // Get the pose estimate for this keyframe using the actual key
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(keys()[keyIdx]);

            // accumulator for Jacobian terms
            gtsam::Matrix16 accumulatedJacobian = gtsam::Matrix16::Zero();
            // Compute error and Jacobian for each point from this keyframe
            for (const std::shared_ptr<Eigen::Vector3d> &scanPointPtr : scanPoints)
            {
                // Transform point from lidar frame to world frame
                const gtsam::Point3
                    l_p(*scanPointPtr),
                    w_p = w_T_imu.compose(imu_T_lidar_).transformFrom(l_p);
                // Point-to-plane distance (signed)
                errorVec(0) += planeNormal_->dot(w_p) - planeNormalOffsetD_;

                if (H)
                {
                    const Eigen::Matrix<double, 1, 3> nT = planeNormal_->transpose();
                    const Eigen::Matrix3d
                        i_R_l = imu_T_lidar_.rotation().matrix(),
                        w_R_i = w_T_imu.rotation().matrix();
                    const gtsam::Point3 i_t_l = imu_T_lidar_.translation();
                    gtsam::Matrix16 D_r_D_X = gtsam::Matrix16::Zero();
                    D_r_D_X.head<3>() = -nT * w_R_i * gtsam::skewSymmetric(i_R_l * l_p + i_t_l);
                    D_r_D_X.tail<3>() = nT;
                    accumulatedJacobian += D_r_D_X;
                }
            }
            if (H)
                (*H)[keyIdx] = accumulatedJacobian / static_cast<double>(totalPoints_);
        }
        errorVec /= static_cast<double>(totalPoints_);
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
