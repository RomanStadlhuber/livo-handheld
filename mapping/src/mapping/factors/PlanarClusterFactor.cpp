#include <mapping/factors/PlanarClusterFactor.hpp>

namespace mapping
{
    PlanarClusterFactor::PlanarClusterFactor(
        const gtsam::KeyVector &keys,
        const gtsam::Pose3 &imu_T_lidar,
        const u_int32_t &clusterId,
        const ClusteringConfig &clusteringConfig)
        : NoiseModelFactor(nullptr, keys),
          imu_T_lidar_(imu_T_lidar),
          clusterId_(clusterId),
          clusteringConfig_(clusteringConfig)
    {
    }

    void PlanarClusterFactor::add(
        std::vector<Eigen::Vector3d &> &lidar_points,
        const gtsam::Key &key,
        const gtsam::Pose3 &world_T_imu)
    {
        lidar_points_[key] = lidar_points;
        world_poses_imu_[key] = world_T_imu;
        numPoints_ += static_cast<double>(lidar_points.size());
        transformPointsToWorldFrame(key, world_T_imu);
    }

    boost::shared_ptr<gtsam::GaussianFactor> PlanarClusterFactor::linearize(const gtsam::Values &x) const
    {
        // computeError will update plane parameters and adaptiveNoiseModel_ if needed
        gtsam::Vector b = computeError(x);

        // If factor is invalid, return empty factor
        if (!isValid_ || !adaptiveNoiseModel_)
            return boost::shared_ptr<gtsam::JacobianFactor>();

        // Get Jacobians for each key
        std::vector<gtsam::Matrix> A = computeJacobians(x);

        // Build the augmented Jacobian matrix [A | b] for WhitenSystem
        // For a JacobianFactor, we need: A * dx = b, where b = -error
        b = -b;

        // Apply whitening to Jacobians and error
        // WhitenSystem modifies matrices in place
        adaptiveNoiseModel_->WhitenSystem(A, b);

        // Build KeyVector and corresponding whitened Jacobian matrices
        // JacobianFactor constructor takes pairs of (key, matrix)
        std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
        terms.reserve(keys().size());
        for (size_t i = 0; i < keys().size(); ++i)
            terms.emplace_back(keys()[i], A[i]);

        return boost::make_shared<gtsam::JacobianFactor>(terms, b);
    }

    gtsam::Vector PlanarClusterFactor::computeError(const gtsam::Values &x) const
    {
        // Build current poses map from values
        std::map<gtsam::Key, gtsam::Pose3> currentPoses;
        for (const gtsam::Key &key : keys())
            currentPoses[key] = x.at<gtsam::Pose3>(key);

        // if plane fit needs to be re-evaluated ..
        if (checkPlaneFit(currentPoses))
        {
            // .. update cached poses ..
            world_poses_imu_ = currentPoses;
            // .. move points to world frame first ..
            for (const gtsam::Key &key : keys())
                transformPointsToWorldFrame(key, world_poses_imu_.at(key));
            // .. then re-evaluate plane parameters
            evaluate(world_poses_imu_);
        }

        gtsam::Vector errorVec = gtsam::Vector::Zero(1);
        if (!isValid_)
        {
            errorVec(0) = 0.0;
            return errorVec;
        }

        double totalError = 0.0;
        for (const auto &[key, points] : world_points_)
        {
            for (const Eigen::Vector3d &pt : points)
            {
                const double pointToPlaneDist = clusterNormal_.dot(pt - clusterCenter_);
                totalError += pointToPlaneDist * pointToPlaneDist;
            }
        }
        totalError /= numPoints_;
        errorVec(0) = totalError;
        return errorVec;
    }

    std::vector<gtsam::Matrix> PlanarClusterFactor::computeJacobians(const gtsam::Values &x) const
    {
        std::vector<gtsam::Matrix> jacobians;
        jacobians.resize(keys().size(), gtsam::Matrix::Zero(1, 6));
        if (!isValid_)
            return jacobians;
        for (size_t i = 0; i < keys().size(); ++i)
        {
            const gtsam::Pose3 w_T_imu = x.at<gtsam::Pose3>(keys()[i]);
            gtsam::Matrix16 accumulatedJacobian = gtsam::Matrix16::Zero();
            // Compute error and Jacobian for each point from this keyframe
            std::vector<Eigen::Vector3d &> l_pts = lidar_points_.at(keys()[i]);
            std::vector<Eigen::Vector3d> w_pts = world_points_.at(keys()[i]);
            for (std::size_t idxPt = 0; idxPt < l_pts.size(); ++idxPt)
            {
                gtsam::Point3 l_p{l_pts[idxPt]}, w_p{w_pts[idxPt]};
                // Transform point from lidar frame to world frame
                const Eigen::Matrix<double, 1, 3> nT = clusterNormal_.transpose();
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
        return jacobians;
    }

    void PlanarClusterFactor::evaluate(const std::map<gtsam::Key, gtsam::Pose3> &world_currentPoses_imu) const
    {
        double numHistoricPoints = numPoints_ - lidar_points_.at(newestKeyframe()).size();
        // --- compute centroid in world coordinates ---
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto &[key, points] : world_points_)
        {
            if (key == newestKeyframe())
                continue;
            for (const Eigen::Vector3d &pt : points)
            {
                centroid += pt;
            }
        }
        centroid /= static_cast<double>(numHistoricPoints);
        if (numHistoricPoints < 4)
        {
            isValid_ = false;
            return;
        }
        centroid /= numHistoricPoints;
        // --- compute plane normal via SVD ---
        Eigen::MatrixXd A(numHistoricPoints, 3);
        size_t idxClusterPt = 0;
        for (const auto &[key, points] : world_points_)
        {
            if (key == newestKeyframe())
                continue;
            for (const Eigen::Vector3d &pt : points)
            {
                A.row(idxClusterPt++) = pt - centroid;
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::Vector3d planeNormal = svd.matrixV().col(2).normalized();

        // Check if plane is valid - all points need to be within threshold distance to plane
        bool isPlaneValid = true;
        double planeThickness = 0.0;
        for (size_t idxPt = 0; idxPt < static_cast<size_t>(numHistoricPoints); idxPt++)
        {
            const double pointToPlaneDist = std::abs(planeNormal.dot(A.row(idxPt)));
            if (pointToPlaneDist > clusteringConfig_.max_plane_thickness)
            {
                isValid_ = false;
                return;
            }
            else
            {
                planeThickness += std::pow(pointToPlaneDist, 2);
            }
        }

        planeThickness /= numHistoricPoints;
        planeThickness_ = planeThickness;
        clusterNormal_ = planeNormal;
        clusterCenter_ = centroid;
        adaptiveNoiseModel_ = gtsam::noiseModel::Isotropic::Variance(1, planeThickness_);
        isValid_ = true;
    }

    void PlanarClusterFactor::transformPointsToWorldFrame(const gtsam::Key &key, const gtsam::Pose3 &world_T_imu) const
    {
        if (world_points_.find(key) == world_points_.end())
            world_points_[key] = std::vector<Eigen::Vector3d>(lidar_points_.at(key).size());
        const gtsam::Pose3 lidar_T_world = imu_T_lidar_.inverse().compose(world_T_imu.inverse());
        const std::vector<Eigen::Vector3d &> &lidarPts = lidar_points_.at(key);
        for (size_t idxPt = 0; idxPt < lidarPts.size(); ++idxPt)
        {
            world_points_[key][idxPt] = lidar_T_world.transformFrom(lidarPts[idxPt]);
        }
    }

    bool PlanarClusterFactor::checkPlaneFit(const std::map<gtsam::Key, gtsam::Pose3> &world_currentPoses_imu) const
    {
        if (!isEvaluated())
            return true;

        for (const auto &[key, previousPose] : world_poses_imu_)
        {
            const gtsam::Pose3 &currentPose = world_currentPoses_imu.at(key);
            const gtsam::Matrix4 poseChange = currentPose.matrix() - previousPose.matrix();
            if (poseChange.norm() >= kThresholdPlaneFit)
                return true;
        }
    }
}