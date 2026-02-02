#include <mapping/factors/PointToPlaneFactor.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace mapping
{

    PointToPlaneFactor::PointToPlaneFactor(
        const gtsam::KeyVector &keys,
        const gtsam::Pose3 &imu_T_lidar,
        const std::unordered_map<gtsam::Key, std::vector<std::shared_ptr<Eigen::Vector3d>>> &scanPointsPerKey,
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
        for (const auto &[key, points] : scanPointsPerKey_)
        {
            totalPoints_ += points.size();
        }
    }

    gtsam::Vector PointToPlaneFactor::unwhitenedError(
        const gtsam::Values &x,
        boost::optional<std::vector<gtsam::Matrix> &> H) const
    {
        if (invalid)
        {
            throw std::runtime_error("Attempting to compute error for an invalid PointToPlaneFactor.");
        }

        auto [errorVec, Hs] = computeErrorAndJacobians(x);
        if (H)
        {
            // Copy computed Jacobians to output
            for (size_t i = 0; i < Hs.size(); ++i)
            {
                (*H)[i] = Hs[i];
            }
        }
        return errorVec;
    }

    boost::shared_ptr<gtsam::GaussianFactor> PointToPlaneFactor::linearize(const gtsam::Values &x) const
    {
        if (invalid)
        {
            throw std::runtime_error("Attempting to linearize an invalid PointToPlaneFactor.");
        }
        auto [errorVec, Hs] = computeErrorAndJacobians(x);
        gtsam::Vector b = -errorVec;
        noiseModel()->WhitenSystem(Hs, b);
        std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
        for (size_t i = 0; i < keys().size(); ++i)
        {
            terms.emplace_back(keys()[i], Hs[i]);
        }
        return boost::make_shared<gtsam::JacobianFactor>(terms, b);
    }

    std::pair<gtsam::Vector, std::vector<gtsam::Matrix>> PointToPlaneFactor::computeErrorAndJacobians(const gtsam::Values &values) const
    {
        const double meanFactor = 1.0 / static_cast<double>(totalPoints_);
        std::vector<gtsam::Matrix> Hs;
        Hs.reserve(keys().size());
        // the returned error is a scalar
        gtsam::Vector errorVec = gtsam::Vector::Zero(1);
        // accumulate mean-squared error and its jacobians w.r.t. the values
        for (const gtsam::Key &key : keys())
        {
            // Get the pose estimate for this keyframe using the actual key
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(key);
            // accumulator for Jacobian terms
            gtsam::Matrix16 Hx = gtsam::Matrix16::Zero();
            // Compute error and Jacobian for each point from this keyframe
            std::vector<std::shared_ptr<Eigen::Vector3d>> scanPoints = scanPointsPerKey_.at(key);
            for (const std::shared_ptr<Eigen::Vector3d> &scanPointPtr : scanPoints)
            {
                // Transform point from lidar frame to world frame
                const gtsam::Point3
                    l_p(*scanPointPtr),
                    w_p = w_T_imu.compose(imu_T_lidar_).transformFrom(l_p);
                const double r_i = planeNormal_->dot(w_p) - planeNormalOffsetD_;
                // Point-to-plane distance (signed)
                errorVec(0) += std::pow(r_i, 2);
                const Eigen::Matrix<double, 1, 3> nT = planeNormal_->transpose();
                const Eigen::Matrix3d
                    i_R_l = imu_T_lidar_.rotation().matrix(),
                    w_R_i = w_T_imu.rotation().matrix();
                const gtsam::Point3 i_t_l = imu_T_lidar_.translation();
                gtsam::Matrix16 D_r_D_X = gtsam::Matrix16::Zero();
                D_r_D_X.head<3>() = 2 * meanFactor * r_i * -nT * w_R_i * gtsam::skewSymmetric(i_R_l * l_p + i_t_l);
                D_r_D_X.tail<3>() = 2 * meanFactor * r_i * nT * w_R_i;
                Hx += D_r_D_X;
            }
            Hs.push_back(Hx);
        }
        errorVec *= meanFactor;
        return {errorVec, Hs};
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
