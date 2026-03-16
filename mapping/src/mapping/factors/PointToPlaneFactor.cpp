/// @file
/// @ingroup factors
#include <mapping/factors/PointToPlaneFactor.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace mapping
{

    gtsam::KeyVector PointToPlaneFactor::buildFullKeys(
        const gtsam::KeyVector &poseKeys,
        boost::optional<gtsam::Key> dtKey,
        boost::optional<gtsam::Key> extrinsicKey)
    {
        gtsam::KeyVector allKeys = poseKeys;
        if (dtKey) allKeys.push_back(*dtKey);
        if (extrinsicKey) allKeys.push_back(*extrinsicKey);
        return allKeys;
    }

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
          clusterId_(clusterId),
          numPoseKeys_(keys.size()),
          dtKey_(boost::none),
          extrinsicKey_(boost::none)
    {
    }

    PointToPlaneFactor::PointToPlaneFactor(
        const gtsam::KeyVector &poseKeys,
        const gtsam::Pose3 &imu_T_lidar,
        const std::map<gtsam::Key, Eigen::Vector3d> &lidar_points,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel,
        const uint32_t &clusterId,
        boost::optional<gtsam::Key> dtKey,
        boost::optional<gtsam::Key> extrinsicKey,
        const std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> &keyframeTwists)
        : NoiseModelFactor(noiseModel, buildFullKeys(poseKeys, dtKey, extrinsicKey)),
          imu_T_lidar_(imu_T_lidar),
          lidar_points_(lidar_points),
          planeNormal_(planeNormal),
          planeCenter_(planeCenter),
          adaptiveNoiseModel_(noiseModel),
          clusterId_(clusterId),
          numPoseKeys_(poseKeys.size()),
          dtKey_(dtKey),
          extrinsicKey_(extrinsicKey),
          keyframeTwists_(keyframeTwists)
    {
    }

    void PointToPlaneFactor::add(
        const gtsam::Key &key,
        const Eigen::Vector3d &lidar_point,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel)
    {
        // Insert pose key before calibration keys
        this->keys_.insert(this->keys_.begin() + numPoseKeys_, key);
        numPoseKeys_++;
        lidar_points_[key] = lidar_point;
        updatePlaneParameters(planeNormal, planeCenter, noiseModel);
    }

    void PointToPlaneFactor::add(
        const gtsam::Key &key,
        const Eigen::Vector3d &lidar_point,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel,
        const std::pair<Eigen::Vector3d, Eigen::Vector3d> &twist)
    {
        add(key, lidar_point, planeNormal, planeCenter, noiseModel);
        keyframeTwists_[key] = twist;
    }

    void PointToPlaneFactor::remove(
        const gtsam::Key &key,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const std::shared_ptr<Eigen::Vector3d> &planeCenter,
        const gtsam::SharedNoiseModel &noiseModel)
    {
        // erase variable key connection (only search among pose keys)
        auto beginIt = keys().begin();
        auto endIt = keys().begin() + numPoseKeys_;
        auto it = std::find(beginIt, endIt, key);
        if (it != endIt)
        {
            keys().erase(it);
            numPoseKeys_--;
        }
        // erase twist for this key if present
        keyframeTwists_.erase(key);
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

        // --- Read calibration values once (before pose loop) ---
        const gtsam::Pose3 imu_T_lidar = extrinsicKey_ && values.exists(*extrinsicKey_)
            ? values.at<gtsam::Pose3>(*extrinsicKey_)
            : imu_T_lidar_;
        const double dt = dtKey_ && values.exists(*dtKey_)
            ? values.at<gtsam::Vector1>(*dtKey_)(0)
            : 0.0;

        // accumulators for calibration Jacobians (summed across all keyframes)
        gtsam::Matrix H_dt_accum = gtsam::Matrix::Zero(1, 1);
        gtsam::Matrix H_ext_accum = gtsam::Matrix::Zero(1, 6);

        // --- pose loop: iterate only over pose keys [0, numPoseKeys_) ---
        for (size_t i = 0; i < numPoseKeys_; ++i)
        {
            const gtsam::Key &key = keys()[i];
            if (!values.exists(key))
            {
                Hs.push_back(gtsam::Matrix16::Zero());
                continue; // needed when called from marginalization -> value was removed but key is still present
            }
            // the plane cluster point associated with this key
            const Eigen::Vector3d &lidar_point = lidar_points_.at(key);
            // Get the pose estimate for this keyframe using the actual key
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(key);

            // Temporal extrapolation: shift IMU pose by dt using precomputed twist
            gtsam::Pose3 w_T_imu_ext = w_T_imu;
            if (dtKey_ && keyframeTwists_.count(key))
            {
                const auto &[omega_k, v_k] = keyframeTwists_.at(key);
                w_T_imu_ext = w_T_imu * gtsam::Pose3(
                    gtsam::Rot3::Expmap(omega_k * dt),
                    v_k * dt);
            }

            // accumulator for Jacobian terms
            gtsam::Matrix16 Hx = gtsam::Matrix16::Zero();
            // Compute error and Jacobian for this keyframes point.
            // Transform point from lidar frame to world frame
            const gtsam::Point3
                l_p(lidar_point),
                w_p = w_T_imu_ext.compose(imu_T_lidar).transformFrom(l_p);
            const double r_i = planeNormal_->dot(w_p - *planeCenter_);
            // Point-to-plane distance (signed)
            errorVec(0) += std::pow(r_i, 2);
            const Eigen::Matrix<double, 1, 3> nT = planeNormal_->transpose();
            const Eigen::Matrix3d
                i_R_l = imu_T_lidar.rotation().matrix(),
                w_R_i = w_T_imu_ext.rotation().matrix();
            const gtsam::Point3 i_t_l = imu_T_lidar.translation();
            Hx.head<3>() = 2 * meanFactor * r_i * -nT * w_R_i * gtsam::skewSymmetric(i_R_l * l_p + i_t_l);
            Hx.tail<3>() = 2 * meanFactor * r_i * nT * w_R_i;
            Hs.push_back(Hx);

            // TODO: formulate residual/jacobian contribution for temporal calibration
            // H_dt_accum += ...;
            // TODO: formulate residual/jacobian contribution for extrinsic calibration
            // H_ext_accum += ...;
        }
        errorVec *= meanFactor;

        // --- Append calibration Jacobians after pose loop ---
        if (dtKey_)
            Hs.push_back(H_dt_accum);
        if (extrinsicKey_)
            Hs.push_back(H_ext_accum);

        return {errorVec, Hs};
    }

    gtsam::NonlinearFactor::shared_ptr PointToPlaneFactor::clone() const
    {
        return boost::make_shared<PointToPlaneFactor>(
            gtsam::KeyVector(keys().begin(), keys().begin() + numPoseKeys_),
            imu_T_lidar_, lidar_points_, planeNormal_, planeCenter_,
            noiseModel(), clusterId_, dtKey_, extrinsicKey_, keyframeTwists_);
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
        // NOTE: keyToMarginalize is always a pose key X(k). Calibration Jacobian
        // entries (for T(0)/E(0)) in Hs are simply ignored here — only the
        // Jacobian for the marginalized pose key is extracted.
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
