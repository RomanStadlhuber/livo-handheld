#include <mapping/factors/PointToPlaneFactor.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace mapping
{

    PointToPlaneFactor::PointToPlaneFactor(
        const gtsam::KeyVector &keys,
        const gtsam::Pose3 &imu_T_lidar,
        const std::map<gtsam::Key, Eigen::Vector3d> &lidar_points,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel,
        const uint32_t &clusterId)
        : NoiseModelFactor(noiseModel, keys),
          imu_T_lidar_(imu_T_lidar),
          lidar_points_(lidar_points),
          planeNormal_(planeNormal),
          planeCenter_(planeCenter),
          adaptiveNoiseModel_(noiseModel),
          clusterId_(clusterId)
    {
    }

    void PointToPlaneFactor::add(
        const gtsam::Key &key,
        const Eigen::Vector3d &lidar_point,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel)
    {
        this->keys_.push_back(key);
        lidar_points_[key] = lidar_point;
        updatePlaneParameters(planeNormal, planeCenter, noiseModel);
    }

    void PointToPlaneFactor::remove(
        const gtsam::Key &key,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel)
    {
        // erase variable key connection
        auto it = std::find(keys().begin(), keys().end(), key);
        if (it != keys().end())
        {
            keys().erase(it);
        }
        // erase lidar points associated with this key
        auto it_map = lidar_points_.find(key);
        if (it_map != lidar_points_.end())
            lidar_points_.erase(it_map);
        updatePlaneParameters(planeNormal, planeCenter, noiseModel);
    }

    void PointToPlaneFactor::updatePlaneParameters(
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel)
    {
        planeNormal_ = planeNormal;
        planeCenter_ = planeCenter;
        adaptiveNoiseModel_ = noiseModel;
    }

    gtsam::Vector PointToPlaneFactor::unwhitenedError(
        const gtsam::Values &x,
        boost::optional<std::vector<gtsam::Matrix> &> H) const
    {
        // reference x and H to silence compiler warnings (usused variables)
        x.size();
        H.has_value();
        throw std::runtime_error(
            "PointToPlaneFactor::unwhitenedError should not be called directly! Use linearize() instead.");
    }

    boost::shared_ptr<gtsam::GaussianFactor> PointToPlaneFactor::linearize(const gtsam::Values &x) const
    {
        if (isInvalid_)
        {
            throw std::runtime_error("Attempting to linearize an invalid PointToPlaneFactor.");
        }
        auto [errorVec, Hs] = computeErrorAndJacobians(x);
        gtsam::Vector b = -errorVec;
        adaptiveNoiseModel_->WhitenSystem(Hs, b);
        std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
        for (size_t i = 0; i < keys().size(); ++i)
        {
            terms.emplace_back(keys()[i], Hs[i]);
        }
        return boost::make_shared<gtsam::JacobianFactor>(terms, b);
    }

    std::pair<gtsam::Vector, std::vector<gtsam::Matrix>> PointToPlaneFactor::computeErrorAndJacobians(const gtsam::Values &values) const
    {
        const double meanFactor = 1.0 / static_cast<double>(lidar_points_.size());
        std::vector<gtsam::Matrix> Hs;
        Hs.reserve(keys().size());
        // the returned error is a scalar
        gtsam::Vector errorVec = gtsam::Vector::Zero(1);
        // accumulate mean-squared error and its jacobians w.r.t. the values
        for (const gtsam::Key &key : keys())
        {
            if (!values.exists(key))
            {
                Hs.push_back(gtsam::Matrix16::Zero());
                continue; // needed when called from marginalization -> value was removed but key is still present
            }
            // the plane cluster point associated with this key
            const Eigen::Vector3d &lidar_point = lidar_points_.at(key);
            // Get the pose estimate for this keyframe using the actual key
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(key);
            // accumulator for Jacobian terms
            gtsam::Matrix16 Hx = gtsam::Matrix16::Zero();
            // Compute error and Jacobian for this keyframes point.
            // Transform point from lidar frame to world frame
            const gtsam::Point3
                l_p(lidar_point),
                w_p = w_T_imu.compose(imu_T_lidar_).transformFrom(l_p);
            const double r_i = planeNormal_->dot(w_p - *planeCenter_);
            // Point-to-plane distance (signed)
            errorVec(0) += std::pow(r_i, 2);
            const Eigen::Matrix<double, 1, 3> nT = planeNormal_->transpose();
            const Eigen::Matrix3d
                i_R_l = imu_T_lidar_.rotation().matrix(),
                w_R_i = w_T_imu.rotation().matrix();
            const gtsam::Point3 i_t_l = imu_T_lidar_.translation();
            Hx.head<3>() = 2 * meanFactor * r_i * -nT * w_R_i * gtsam::skewSymmetric(i_R_l * l_p + i_t_l);
            Hx.tail<3>() = 2 * meanFactor * r_i * nT * w_R_i;
            Hs.push_back(Hx);
        }
        errorVec *= meanFactor;
        return {errorVec, Hs};
    }

    gtsam::NonlinearFactor::shared_ptr PointToPlaneFactor::clone() const
    {
        return boost::make_shared<PointToPlaneFactor>(
            keys(), imu_T_lidar_, lidar_points_, planeNormal_, planeCenter_, noiseModel(), clusterId_);
    }

    void PointToPlaneFactor::print(const std::string &s, const gtsam::KeyFormatter &keyFormatter) const
    {
        std::cout << s << "PointToPlaneFactor (cluster id: " << clusterId_ << ") connecting keys: ";
        for (const auto &key : keys())
        {
            std::cout << keyFormatter(key) << " ";
        }
        std::cout << "\n\tCluster Id: " << clusterId_ << "\n"
                  << "\tPlane normal: [" << planeNormal_->transpose() << "]\n"
                  << "\tPlane center: [" << planeCenter_->transpose() << "]\n"
                  << "\tTotal points: " << lidar_points_.size() << "\n"
                  << "\tNoise model: ";
        adaptiveNoiseModel_->print("");
    }

    gtsam::LinearContainerFactor::shared_ptr PointToPlaneFactor::createMarginalizationFactor(
        const gtsam::Values &values,
        const gtsam::Key &keyToMarginalize) const
    {
        auto [r, Hs] = computeErrorAndJacobians(values);
        // need to flip the same way as in linearize() to build the system
        // H * dx = -r
        gtsam::Vector b = -r;
        adaptiveNoiseModel_->WhitenSystem(Hs, b);
        gtsam::Matrix H = gtsam::Matrix16::Zero();
        for (size_t i = 0; i < Hs.size(); ++i)
            if (keys()[i] == keyToMarginalize)
            {
                H = Hs[i];
                break;
            }
        gtsam::JacobianFactor::shared_ptr J = boost::make_shared<gtsam::JacobianFactor>(keyToMarginalize, H, b);
        return boost::make_shared<gtsam::LinearContainerFactor>(J, values);
    }

} // namespace mapping
