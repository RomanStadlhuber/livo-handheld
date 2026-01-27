#include <mapping/MappingSystem.hpp>

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <unordered_map>
#include <memory>

namespace mapping
{

    // Symbol shortcuts for factor graph keys
    using gtsam::symbol_shorthand::B;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;

    open3d::geometry::PointCloud Scan2PCD(
        const std::shared_ptr<LidarData> &lidar_data,
        double minPointDist,
        double maxPointDist)
    {
        open3d::geometry::PointCloud pcd;
        size_t point_num = lidar_data->points.size();
        pcd.points_.reserve(point_num);
        for (size_t i = 0; i < point_num; ++i)
        {
            const Eigen::Vector3d &pt = lidar_data->points[i];
            const double dist = pt.norm();
            if (dist < minPointDist || dist > maxPointDist)
                continue;
            pcd.points_.push_back(lidar_data->points[i]);
        }
        return pcd;
    }

    MappingSystem::MappingSystem()
        : MappingSystem(MappingConfig())
    {
    }

    MappingSystem::MappingSystem(const MappingConfig &config)
        : systemState_(SystemState::Initializing),
          keyframeCounter_(0),
          tLastImu_(0.0),
          clusterIdCounter_(0),
          config_(config),
          imu_T_lidar_(config.extrinsics.imu_T_lidar.toPose3()),
          lidarTimeOffset_(0.0)
    {
        gtsam::ISAM2Params smootherParams;
        smootherParams.optimizationParams = gtsam::ISAM2GaussNewtonParams();
        smootherParams.findUnusedFactorSlots = true;
        smoother_ = gtsam::IncrementalFixedLagSmoother(static_cast<double>(config_.backend.sliding_window_size), smootherParams);
        // Initialize IMU preintegration parameters
        auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        // noise values from:
        // https://github.com/RomanStadlhuber/mid360_docker/blob/main/mid360_runner/config/mapping_mid360.yaml#L13
        constexpr double accelerometerNoise = 0.1;
        constexpr double gyroscopeNoise = 0.1;
        params->accelerometerCovariance = gtsam::I_3x3 * accelerometerNoise;
        params->gyroscopeCovariance = gtsam::I_3x3 * gyroscopeNoise;
        // bias value from:
        // https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/include/use-ikfom.hpp#L35
        params->integrationCovariance = gtsam::I_3x3 * 1e-4;
        const gtsam::Vector6 commonBias{gtsam::Vector6::Ones() * 0.0001};
        gtsam::imuBias::ConstantBias priorImuBias{commonBias};
        preintegrator_ = gtsam::PreintegratedCombinedMeasurements(params, priorImuBias);
    }

    void MappingSystem::setConfig(const MappingConfig &config)
    {
        config_ = config;
        imu_T_lidar_ = config_.extrinsics.imu_T_lidar.toPose3();
    }

    void MappingSystem::feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp)
    {
        imuBuffer_[timestamp] = imu_data;
    }

    void MappingSystem::feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp)
    {
        lidarBuffer_[timestamp] = lidar_data;
    }

    void MappingSystem::update()
    {
        switch (systemState_)
        {
        case SystemState::Initializing:
        {
            const double maxBufferTime = lidarBuffer_.rbegin()->first;
            if (maxBufferTime >= config_.backend.init_time_window)
            {
                initializeSystem();
                std::cout << "Initialization complete. Switching to tracking state." << std::endl;
                systemState_ = SystemState::Tracking;
            }
            break;
        }
        case SystemState::Tracking:
        {
            track();
            break;
        }
        case SystemState::Recovery:
        {
            while (true)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            break;
        }
        }
#ifndef DISABLEVIZ
        visualizer.refreshWindow();
#endif
    }

    void MappingSystem::initializeSystem()
    {
        // Lock the buffers for accessing values
        std::lock_guard<std::mutex> lockImuBuffer(mtxImuBuffer_), lockLidarBuffer(mtxLidarBuffer_);
        const double tInit = lidarBuffer_.rbegin()->first;
        // Upper bound key for all imu samples up to and including tInit
        auto imuBufferEndIt = imuBuffer_.upper_bound(tInit);
        auto lidarBufferEndIt = lidarBuffer_.upper_bound(tInit);
        // Gravity direction is obtained from mean accelerometer measurement
        const double numImuSamples = static_cast<double>(std::distance(imuBuffer_.begin(), imuBufferEndIt));
        Eigen::Vector3d accMean = Eigen::Vector3d::Zero();
        for (auto it = imuBuffer_.begin(); it != imuBufferEndIt; ++it)
            accMean += it->second->acceleration;
        accMean /= numImuSamples;
        std::cout << "Mean accelerometer measurement during initialization: " << std::endl
                  << accMean.transpose() << std::endl;

        // Build gravity-aligned global reference frame (only roll & pitch are observable)
        Eigen::Vector3d zAxis = accMean.normalized();
        // Orthogonal projection of global x-axis onto plane normal to zAxis
        // x_orthog = (I - zz^T) * x
        Eigen::Vector3d xAxis = ((Eigen::Matrix3d::Identity() - zAxis * zAxis.transpose()) * Eigen::Vector3d::UnitX()).normalized();
        Eigen::Vector3d yAxis = zAxis.cross(xAxis);

        // Build initial orientation matrix
        Eigen::Matrix3d w_R_i0;
        w_R_i0.col(0) = xAxis;
        w_R_i0.col(1) = yAxis;
        w_R_i0.col(2) = zAxis;
        w_R_i0.transposeInPlace(); // rotation from initial IMU frame to world frame

        // Full initial pose
        gtsam::Pose3 w_T_i0{gtsam::Rot3(w_R_i0), gtsam::Point3(0, 0, 0)};

        // Create new submap with all initial lidar scans
        open3d::geometry::PointCloud newSubmap;
        gtsam::Pose3 w_T_l0{w_T_i0.compose(imu_T_lidar_)};
        for (auto it = lidarBuffer_.begin(); it != lidarBufferEndIt; ++it)
        {
            open3d::geometry::PointCloud pcdScan = Scan2PCD(it->second, config_.point_filter.min_distance, config_.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScan = pcdScan.VoxelDownSample(config_.lidar_frontend.voxel_size);
            ptrPcdScan->Transform(w_T_l0.matrix());
            newSubmap += *ptrPcdScan;
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(config_.lidar_frontend.voxel_size);
        const uint32_t idxNewKF = createKeyframeSubmap(w_T_l0, tLastImu_, ptrNewSubmapVoxelized);
        std::vector<SubmapIdxPointIdx> clusterPoints; // empty for initial keyframe
        clusterPoints.reserve(keyframeSubmaps_.at(idxNewKF)->points_.size());
        for (size_t i = 0; i < keyframeSubmaps_.at(idxNewKF)->points_.size(); ++i)
            clusterPoints.emplace_back(idxNewKF, static_cast<int>(i));
        createNewClusters(idxNewKF, clusterPoints);
        // Clear lidar buffer
        lidarBuffer_.clear();

        // Acceleration bias is unobservable because the mean value is used for gravity alignment
        // Gyro bias is the mean of all gyro measurements (because no rotation is assumed)
        Eigen::Vector3d gyroBiasMean = Eigen::Vector3d::Zero();
        for (auto it = imuBuffer_.begin(); it != imuBufferEndIt; ++it)
            gyroBiasMean += it->second->angular_velocity;
        gyroBiasMean /= numImuSamples;

        // Variances are computed w.r.t. the mean for acceleration and 0 for gyro measurements
        double accVariance = 0.0, gyroVariance = 0.0;
        for (auto it = imuBuffer_.begin(); it != imuBufferEndIt; ++it)
        {
            accVariance += (it->second->acceleration - accMean).squaredNorm();
            gyroVariance += (it->second->angular_velocity).squaredNorm();
        }
        accVariance /= (numImuSamples - 1.0);
        gyroVariance /= (numImuSamples - 1.0);
        preintegrator_.params()->accelerometerCovariance = gtsam::I_3x3 * accVariance;
        preintegrator_.params()->gyroscopeCovariance = gtsam::I_3x3 * gyroVariance;

        // Set bias (use existing acceleration bias)
        gtsam::imuBias::ConstantBias priorImuBias{preintegrator_.biasHat().accelerometer(), gyroBiasMean};
        preintegrator_.resetIntegrationAndSetBias(priorImuBias);

        // Construct Navigation State prior (mean & covariance)
        resetNewFactors();
        gtsam::Vector6 priorPoseSigma;
        priorPoseSigma << 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3;

        // Pose prior should be certain
        newSmootherFactors_.addPrior(X(idxNewKF), w_T_i0, gtsam::noiseModel::Diagonal::Sigmas(priorPoseSigma));
        newSmootherFactors_.addPrior(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Isotropic::Sigma(3, 1e-6));
        // Bias prior, very noisy
        newSmootherFactors_.addPrior(B(idxNewKF), priorImuBias, gtsam::noiseModel::Isotropic::Sigma(6, 3.0 * std::max(accVariance, gyroVariance)));
        newValues_.insert(X(idxNewKF), w_T_i0);
        newValues_.insert(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()));
        newValues_.insert(B(idxNewKF), priorImuBias);
        w_X_curr_ = gtsam::NavState(w_T_i0, gtsam::Vector3::Zero());
        const double kfInit = static_cast<double>(idxNewKF);

        // Set index of all values
        for (auto const &val : newValues_)
            newSmootherIndices_[val.key] = kfInit;

        // Clear imu buffer and store timestamp of last IMU reading
        tLastImu_ = imuBufferEndIt->first;
        imuBuffer_.clear();

#ifndef DISABLEVIZ
        visualizer.createWindow(); // create window & add world frame
        visualizer.addSubmap(idxNewKF, w_T_i0.matrix(), ptrNewSubmapVoxelized);
        visualizer.waitForSpacebar();
#endif
    }

    gtsam::NavState MappingSystem::preintegrateIMU()
    {
        // Lock the buffers for accessing values
        std::unique_lock<std::mutex> lockImuBuffer(mtxImuBuffer_);

        // Perform preintegration to decide whether a new keyframe is needed
        for (auto imuIt = imuBuffer_.begin(); imuIt != imuBuffer_.end(); ++imuIt)
        {
            const auto [timestamp, u] = *imuIt;
            const double dt = imuIt == imuBuffer_.begin() ? timestamp - tLastImu_ : timestamp - std::prev(imuIt)->first;
            // Integrate measurement
            preintegrator_.integrateMeasurement(u->acceleration, u->angular_velocity, dt);
        }

        // Save last imu timestamp and clear the buffer
        tLastImu_ = imuBuffer_.rbegin()->first;
        imuBuffer_.clear();
        lockImuBuffer.unlock();

        const gtsam::NavState w_X_propagated = preintegrator_.predict(w_X_curr_, currBias_);
        return w_X_propagated;
    }

    void MappingSystem::undistortScans()
    {
        // Iterator to the newest lidar scan that can still be processed (older than latest imu timestamp)
        std::unique_lock<std::mutex> lockLidarBuffer(mtxLidarBuffer_);

        // Undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation
        // Delta pose to last submap at preintegrated state / last IMU time
        gtsam::Pose3 kf_T_prop = lastKeyframePose().inverse().compose(w_X_curr_.pose().compose(imu_T_lidar_));

        const double tLastKeyframe = keyframeTimestamps_.rbegin()->second;
        const double dtPropToKeyframe = tLastImu_ - tLastKeyframe;

        // Undistort between individual scans
        gtsam::Pose3 deltaPoseLastScanToKeyframe = gtsam::Pose3::Identity();
        for (auto it = lidarBuffer_.begin(); it != lidarBuffer_.end(); ++it)
        {
            const auto [tScan, scan] = *it;
            const double dtScanToKeyframe = tScan - tLastKeyframe;

            // Pose delta of this scan to last keyframe
            // Compute pose delta to last submap at current scan time (extrapolate with constant velocity)
            // Linear and angular velocity are obtained from pose delta over preintegration time
            const gtsam::Vector3 angVel = gtsam::Rot3::Logmap(kf_T_prop.rotation()) / dtPropToKeyframe;
            const gtsam::Vector3 linVel = kf_T_prop.translation() / dtPropToKeyframe;

            // Apply scan to keyframe delta time to get pose delta of the scan w.r.t. the last keyframe
            gtsam::Rot3 kf_R_scan = gtsam::Rot3::Expmap(angVel * dtScanToKeyframe);
            gtsam::Point3 kf_P_scan = linVel * dtScanToKeyframe;
            gtsam::Pose3 kf_T_scan{kf_R_scan, kf_P_scan};

            // Pose & time delta from this to last scan
            gtsam::Pose3 lastScan_T_currScan = deltaPoseLastScanToKeyframe.between(kf_T_scan);

            // Undistort current scan
            for (std::size_t i = 0; i < scan->points.size(); ++i)
            {
                const Eigen::Vector3d pt = scan->points[i];
                const double dt = scan->offset_times[i];
                const gtsam::Vector3 scanAngVel = gtsam::Rot3::Logmap(lastScan_T_currScan.rotation());

                // Presumed rotation and translation that the point should have undergone during scan time
                gtsam::Rot3 scan_R_pt = gtsam::Rot3::Expmap(scanAngVel * dt);
                gtsam::Point3 scan_P_pt = lastScan_T_currScan.translation() * dt;
                gtsam::Pose3 scan_T_pt{scan_R_pt, scan_P_pt};

                // Undistort point
                const Eigen::Vector3d ptUndistorted = scan_T_pt.transformFrom(pt);
                scan->points[i] = ptUndistorted;
            }

            // Convert scan to pointcloud, voxelize, transform to keyframe pose and add to submap
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, config_.point_filter.min_distance, config_.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(config_.lidar_frontend.voxel_size);
            gtsam::Pose3 scanPoseInWorld = lastKeyframePose().compose(kf_T_scan);
            // ptrPcdScanVoxelized->Transform(scanPoseInWorld.matrix());

            // Buffer undistorted scan
            bufferScan(kf_T_scan, ptrPcdScanVoxelized);
            deltaPoseLastScanToKeyframe = kf_T_scan;
        }

        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // Erase processed raw scan data and release buffer lock
        lidarBuffer_.clear();
        lockLidarBuffer.unlock();
    }

    std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd> MappingSystem::planeFitSVD(
        const std::vector<Eigen::Vector3d> &points,
        double planarityThreshold,
        double linearityThreshold) const
    {
        const size_t numPoints = points.size();

        // Compute centroid
        Eigen::Vector3d planeCenter = Eigen::Vector3d::Zero();
        for (const auto &pt : points)
            planeCenter += pt;
        planeCenter /= static_cast<double>(numPoints);

        // Build centered points matrix (Nx3)
        Eigen::MatrixXd planePoints(numPoints, 3);
        for (size_t i = 0; i < numPoints; ++i)
            planePoints.row(i) = (points[i] - planeCenter).transpose();

        // SVD to find plane normal (smallest singular vector)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(planePoints, Eigen::ComputeThinV);
        Eigen::Vector3d planeNormal = svd.matrixV().col(2).normalized();

        // Get singular values: σ₁ >= σ₂ >= σ₃
        const Eigen::VectorXd singularValues = svd.singularValues();
        const double sigma1 = singularValues(0);
        const double sigma2 = singularValues(1);
        const double sigma3 = singularValues(2);

        // Check validity:
        // 1. Planarity: σ₃/σ₂ <= threshold (points are flat, not a blob)
        // 2. Non-linearity: σ₂/σ₁ >= threshold (points have 2D spread, not a line)
        const bool isPlanar = (sigma2 > 1e-10) && (sigma3 / sigma2) <= planarityThreshold;
        const bool notLinear = (sigma1 > 1e-10) && (sigma2 / sigma1) >= linearityThreshold;
        const bool isValid = isPlanar && notLinear;

        return {isValid, planeNormal, planeCenter, planePoints};
    }

    bool MappingSystem::trackScanPointsToClusters(const uint32_t &idxKeyframe)
    {
        // Search for same-plane point clusters among all keyframes
        std::vector<int> knnIndices(config_.lidar_frontend.knn_neighbors);
        knnIndices.reserve(config_.lidar_frontend.knn_neighbors);
        std::vector<double> knnDists(config_.lidar_frontend.knn_neighbors);
        knnDists.reserve(config_.lidar_frontend.knn_neighbors);
        std::size_t validTracks = 0;
        // TODO: remove stopwatch after slowdown was diagnosed
        auto stopwatchKNNStart = std::chrono::high_resolution_clock::now();
        // KD-Tree of the current submap, used for cluster tracking
        const open3d::geometry::KDTreeFlann kdTree{*keyframeSubmaps_[idxKeyframe]};
        // project each cluster point onto the current keyframe and try to find the 5 nearest neighbors
        for (auto const &cluster : clusters_)
        {
            auto const &[clusterId, clusterPointIdxs] = cluster;
            // skip tracking invalid clusters (i.e. where tracking was already lost)
            if (clusterPointIdxs.size() > config_.lidar_frontend.clustering.min_points && clusterValidity_[clusterId] == false)
                continue;
            auto const &[idxClusterKF, idxSubmapPt] = *clusterPointIdxs.rbegin(); // get the cluster point from the latest keyframe
            const Eigen::Vector3d &world_clusterPt = keyframeSubmaps_[idxClusterKF]->points_[idxSubmapPt];
            const int knnFound = kdTree.SearchKNN(
                world_clusterPt,
                // config_.lidar_frontend.knn_radius,
                config_.lidar_frontend.knn_neighbors,
                knnIndices,
                knnDists);
            if (knnFound < config_.lidar_frontend.knn_neighbors)
                continue; // not enough neighbors found, skip this cluster
            const double nKnn = static_cast<double>(knnFound);
            // collect KNN points and fit a plane
            std::vector<Eigen::Vector3d> knnPoints;
            knnPoints.reserve(knnFound);
            for (int i = 0; i < knnFound; ++i)
                knnPoints.push_back(keyframeSubmaps_[idxKeyframe]->points_[knnIndices[i]]);
            const auto [planeValid, planeNormal, knnCenter, knnPointsMat] = planeFitSVD(knnPoints);
            // skip if plane fit is degenerate (collinear or non-planar points)
            if (!planeValid)
                continue;
            // validate plane fit by checking point-to-plane distances
            // (+ compute plane thickness during check)
            bool validPlane = true;
            double planeThickness = 0.0;
            for (int i = 0; i < knnFound; ++i)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(knnPointsMat.row(i)));
                if (pointToPlaneDist > config_.lidar_frontend.clustering.max_plane_thickness)
                {
                    validPlane = false;
                    break;
                }
                planeThickness += std::pow(pointToPlaneDist, 2.0);
            }
            if (!validPlane)
                continue;
            planeThickness /= nKnn;
            // associate the nearest point as a track to this cluster
            std::size_t idxKnnNearest = knnIndices[0];
            for (std::size_t i = 1; i < knnIndices.size(); ++i)
                if (knnDists[i] < knnDists[idxKnnNearest])
                    idxKnnNearest = knnIndices[i];
            addPointToCluster(clusterId, {idxKeyframe, idxKnnNearest}, planeThickness);
            validTracks++;
            knnIndices.clear();
            knnDists.clear();
        }
        auto stopwatchKNNEnd = std::chrono::high_resolution_clock::now();
        auto durationKNN = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchKNNEnd - stopwatchKNNStart).count();
        std::cout << "::: [DEBUG] KNN search and cluster association took " << durationKNN << " ms :::" << std::endl;
        std::size_t numValidClusters = 0;
        // compute the actual cluster parameters (updated centroid, normal, thickness) based on all associated points
        for (auto const &cluster : clusters_)
        {
            /**
             * NOTE: mark clusters invalid and skip parameters if
             * - they are premature (not enough keyframe associations yet)
             * - they are not associated with the current keyframe (tracking lost)
             */
            auto const &[clusterId, clusterPointIdxs] = cluster;
            if (
                clusterPointIdxs.size() < config_.lidar_frontend.clustering.min_points // premature
                || clusterPointIdxs.find(idxKeyframe) == clusterPointIdxs.end())       // no association with current KF
            {
                clusterValidity_[clusterId] = false;
                continue;
            }
            // collect all points associated with this cluster
            std::vector<Eigen::Vector3d> clusterPoints;
            clusterPoints.reserve(clusterPointIdxs.size());
            for (auto const &pointIdxPair : clusterPointIdxs)
            {
                const auto &[idxSubmap, idxPoint] = pointIdxPair;
                clusterPoints.push_back(keyframeSubmaps_[idxSubmap]->points_[idxPoint]);
            }
            // fit plane to all points
            const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat] = planeFitSVD(clusterPoints);
            // skip if plane fit is degenerate (collinear or non-planar points)
            if (!planeValid)
            {
                clusterValidity_[clusterId] = false;
                continue;
            }
            // average plane thickness from historical tracks
            double planeThicknessCovariance = 0.0;
            // MSC-LIO, Eq. 19 (plane thickness covariance and adaptive sigma)
            for (const double &thickness : clusterPlaneThicknessHistory_[clusterId])
                planeThicknessCovariance += std::pow(thickness, 2.0);
            planeThicknessCovariance /= static_cast<double>(clusterPlaneThicknessHistory_[clusterId].size());
            const double adaptiveSigma = std::pow(0.5 * planeThicknessCovariance, 0.25);
            // update cached cluster parameters
            clusterCenters_[clusterId] = std::make_shared<Eigen::Vector3d>(clusterCenter);
            clusterNormals_[clusterId] = std::make_shared<Eigen::Vector3d>(planeNormal);
            clusterPlaneThickness_[clusterId] = planeThicknessCovariance;
            clusterSigmas_[clusterId] = adaptiveSigma;
            // check if all point-to-plane distances pass the 6-sigma test
            clusterValidity_[clusterId] = true; // reset validity
            for (long int i = 0; i < clusterPointsMat.rows(); ++i)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(clusterPointsMat.row(i)));
                if (pointToPlaneDist > 3.0 * adaptiveSigma)
                {
                    // std::cout << "::: [WARNING] Cluster " << clusterId << " failed 6-sigma test with point-to-plane distance " << pointToPlaneDist << " :::" << std::endl;
                    clusterValidity_[clusterId] = false;
                    break;
                }
            }
            if (clusterValidity_[clusterId])
                numValidClusters++;
        }
        std::cout << "::: [INFO] keyframe " << idxKeyframe << " had " << validTracks << " tracks and " << numValidClusters << " valid clusters :::" << std::endl;
        return validTracks > 0;
    }

    void MappingSystem::createNewClusters(const uint32_t &idxKeyframe, std::vector<SubmapIdxPointIdx> &clusterPoints)
    {
        std::size_t numCreated = 0, numRejected = 0;
        const open3d::geometry::KDTreeFlann kdTree{*keyframeSubmaps_[idxKeyframe]};
        std::vector<int> knnIndices(config_.lidar_frontend.knn_neighbors);
        std::vector<double> knnDists(config_.lidar_frontend.knn_neighbors);

        for (auto const &[idxSubmap, idxPoint] : clusterPoints)
        {
            // find nearest neighbors for the candidate point
            const Eigen::Vector3d &queryPoint = keyframeSubmaps_[idxKeyframe]->points_[idxPoint];
            const int knnFound = kdTree.SearchKNN(
                queryPoint,
                config_.lidar_frontend.knn_neighbors,
                knnIndices,
                knnDists);

            if (knnFound < config_.lidar_frontend.knn_neighbors)
            {
                numRejected++;
                continue; // not enough neighbors, skip this point
            }

            // collect neighbor points and fit a plane
            std::vector<Eigen::Vector3d> knnPoints;
            knnPoints.reserve(knnFound);
            for (int i = 0; i < knnFound; ++i)
                knnPoints.push_back(keyframeSubmaps_[idxKeyframe]->points_[knnIndices[i]]);

            const auto [planeValid, planeNormal, planeCenter, planePointsMat] = planeFitSVD(knnPoints);

            // skip if plane fit is degenerate (collinear or non-planar points)
            if (!planeValid)
            {
                numRejected++;
                continue;
            }

            // validate plane fit by checking all point-to-plane distances
            bool validThickness = true;
            for (int i = 0; i < knnFound; ++i)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(planePointsMat.row(i)));
                if (pointToPlaneDist > config_.lidar_frontend.clustering.max_plane_thickness)
                {
                    validThickness = false;
                    break;
                }
            }

            if (!validThickness)
            {
                numRejected++;
                continue;
            }

            // plane fit is valid, create new cluster
            std::map<u_int32_t, std::size_t> newCluster({{idxKeyframe, idxPoint}});
            auto const clusterId = clusterIdCounter_++;
            clusters_.emplace(clusterId, newCluster);
            clusterValidity_.emplace(clusterId, false);
            numCreated++;

            knnIndices.clear();
            knnDists.clear();
        }
        std::cout << "::: [INFO] Created " << numCreated << " new clusters from keyframe " << idxKeyframe
                  << " (" << numRejected << " rejected due to plane fit) :::" << std::endl;
    }

    void MappingSystem::addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness)
    {
        auto const &[idxSubmap, idxPoint] = pointIdx; // destructure to make it clearer what's going on
        clusters_[clusterId][idxSubmap] = idxPoint;
        clusterPlaneThicknessHistory_[clusterId].push_back(planeThickness);
    }

    void MappingSystem::removeKeyframeFromClusters(const u_int32_t &idxKeyframe)
    {
        for (auto &[clusterId, clusterPoints] : clusters_)
        {
            auto itPoint = clusterPoints.find(idxKeyframe);
            if (itPoint != clusterPoints.end())
            {
                clusterPoints.erase(itPoint);
            }
        }
    }

    void MappingSystem::pruneClusters(const uint32_t &idxKeyframe)
    {
        if (idxKeyframe < static_cast<size_t>(config_.backend.sliding_window_size))
            return;
        std::set<ClusterId> clustersToErase;
        for (auto const &cluster : clusters_)
        {
            const auto &[clusterId, clusterPoints] = cluster;
            // erase if cluster has enough points but is invalid
            if (!clusterValidity_.at(clusterId) && clusterPoints.size() > config_.lidar_frontend.clustering.min_points)
            {
                clustersToErase.insert(clusterId);
            }
            else if (clusterPoints.rbegin()->first != idxKeyframe)
            {
                // erase if the cluster is not associated with the latest keyframe
                clustersToErase.insert(clusterId);
            }
        }
        for (const auto &clusterId : clustersToErase)
        {
            clusters_.erase(clusterId);
            clusterValidity_.erase(clusterId);
            clusterCenters_.erase(clusterId);
            clusterNormals_.erase(clusterId);
            clusterPlaneThickness_.erase(clusterId);
            clusterSigmas_.erase(clusterId);
            clusterPlaneThicknessHistory_.erase(clusterId);
        }
        std::cout << "::: [INFO] Pruned " << clustersToErase.size() << " clusters, "
                  << clusters_.size() << " clusters remain :::" << std::endl;
    }
    void MappingSystem::createAndUpdateFactors()
    {
        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            // skip if cluster is invalid or too small
            if (!clusterValidity_.at(clusterId) || clusterPoints.size() < config_.lidar_frontend.clustering.min_points) // need at least 2 keyframes to factor in a constraint
                continue;
            const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
            const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
            const double adaptiveSigma = clusterSigmas_[clusterId];

            // 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
            gtsam::KeyVector keys;
            keys.reserve(clusterPoints.size());
            std::unordered_map<uint32_t, gtsam::Key> keyframeToGraphKeyMapping;
            // 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
            std::unordered_map<gtsam::Key, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey;
            size_t totalPoints = 0;
            for (auto const &[idxKeyframe, idxPoint] : clusterPoints)
            {
                const gtsam::Key key = X(idxKeyframe);
                keys.push_back(key);
                keyframeToGraphKeyMapping[idxKeyframe] = key;
                // scan points are passed to factor in world frame
                scanPointsPerKey[key].push_back(std::make_shared<Eigen::Vector3d>(
                    // transform point in keyframe from world frame to lidar frame
                    keyframePoses_[idxKeyframe]->transformTo(keyframeSubmaps_[idxKeyframe]->points_[idxPoint])));
                totalPoints++;
            }

            // Noise model for cluster point factor
            gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
            const PointToPlaneFactor::shared_ptr ptpFactor = boost::make_shared<PointToPlaneFactor>(
                keys,
                imu_T_lidar_,
                scanPointsPerKey,
                clusterNormal,
                clusterNormal->dot(*clusterCenter),
                noiseModel,
                clusterId);

            const gtsam::NonlinearFactorGraph &smootherFactors = smoother_.getFactors();
            bool isFactorAdded = true;
            for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
            {
                const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                {
                    // mark existing factor for removal
                    factorsToRemove_.push_back(gtsam::Key{factorKey});
                    isFactorAdded = false;
                    break;
                }
            }

            // TODO: remove this when bad factors are fixed
            // (print when the factor is a new add)
            if (isFactorAdded)
                ptpFactor->print();

            newSmootherFactors_.add(ptpFactor);
        }
        std::cout << "::: [INFO] adding " << newSmootherFactors_.size() - factorsToRemove_.size()
                  << " LiDAR factors, updating " << factorsToRemove_.size() << " :::" << std::endl;
    }

    void MappingSystem::track()
    {
        const gtsam::NavState w_X_propagated = preintegrateIMU();
        const double
            positionDiff = (w_X_propagated.pose().translation() - lastKeyframePose().translation()).norm(),
            angleDiff = (lastKeyframePose().rotation().between(w_X_propagated.pose().rotation())).axisAngle().second;
        w_X_curr_ = w_X_propagated;
        // undistort all scans and move them to the scanBuffer
        const double tLastScan = lidarBuffer_.rbegin()->first;
        undistortScans();
        // check if a new keyframe is needed
        if (
            positionDiff < config_.lidar_frontend.keyframe.thresh_distance                    // translation threshold
            && angleDiff < config_.lidar_frontend.keyframe.thresh_angle                       // angle threshold
            && scansSinceLastKeyframe_ < config_.lidar_frontend.keyframe.thresh_elapsed_scans // number of elapsed scans since last keyframe
        )
            return;
        std::cout << "::: [INFO] identified keyframe, creating new submap :::" << std::endl;
        std::cout << "::: [DEBUG] position diff: " << positionDiff
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_distance << "), angle diff: " << angleDiff
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_angle << "), scans elapsed: " << scansSinceLastKeyframe_
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_elapsed_scans << ") :::" << std::endl;
        // Create new keyframe
        // Merge buffered scans together & create new keyframe submap
        open3d::geometry::PointCloud newSubmap;
        /**
         * NOTE: the scan buffer contains the undistorted scan pointclouds in their own scan origin frame,
         * along with the pose that the scan has w.r.t. the last keyframe.
         * In order to build the new submap at the new keyframe origin, all buffered scans need to be transformed
         * by their pose w.r.t. the new keyframe first.
         *
         * This means inverting the pose of the last scan to the last keyframe, then composing it with each scan's
         * pose to the last keyframe to get the pose of each scan w.r.t. the new keyframe.
         *
         * It is important that the scans in the buffer are not transformed before this.
         */
        const gtsam::Pose3 newKf_T_lastKf = scanBuffer_.rbegin()->kf_T_scan->inverse();
        for (auto reverseIt = scanBuffer_.rbegin(); reverseIt != scanBuffer_.rend(); ++reverseIt)
        {
            const ScanBuffer &scan = *reverseIt;
            // pose of the scan w.r.t. the new keyframe
            const gtsam::Pose3 newKf_T_scan = newKf_T_lastKf.compose(*(scan.kf_T_scan));
            // move undistorted pcd to the origin of the new keyframe
            scan.pcd->Transform(newKf_T_scan.matrix());
            newSubmap += *(scan.pcd);
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(config_.lidar_frontend.voxel_size);
        const gtsam::Pose3 world_T_lidar = w_X_curr_.pose().compose(imu_T_lidar_);
        const uint32_t idxKeyframe = createKeyframeSubmap(world_T_lidar, tLastScan, ptrNewSubmapVoxelized);
#ifndef DISABLEVIZ
        visualizer.addSubmap(idxKeyframe, world_T_lidar.matrix(), ptrNewSubmapVoxelized);
        visualizer.waitForSpacebar();
#endif

        const bool isTracking = trackScanPointsToClusters(idxKeyframe);
        pruneClusters(idxKeyframe);
        if (!isTracking)
        {
            std::cout << "::: [ERROR] lost tracking at keyframe " << idxKeyframe << " :::" << std::endl;
            systemState_ = SystemState::Recovery;
            return;
        }
        // Add variables for the active keyframe
        newValues_.insert(X(idxKeyframe), w_X_curr_.pose());
        newValues_.insert(V(idxKeyframe), w_X_curr_.v());
        newValues_.insert(B(idxKeyframe), currBias_);
        summarizeClusters();
        auto stopwatchFactorsStart = std::chrono::high_resolution_clock::now();
        // NOTE: will internally update factorsToRemove to drop outdated smart factors
        // the outdated factors will be replaced by extended ones with additional tracks
        createAndUpdateFactors();
        auto stopwatchFactorsEnd = std::chrono::high_resolution_clock::now();
        auto durationFactors = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchFactorsEnd - stopwatchFactorsStart).count();
        std::cout << "::: [DEBUG] factor creation and update took " << durationFactors << " ms :::" << std::endl;
        // Preintegration factor
        newSmootherFactors_.add(
            gtsam::CombinedImuFactor(
                X(idxKeyframe - 1), V(idxKeyframe - 1),
                X(idxKeyframe), V(idxKeyframe),
                B(idxKeyframe - 1), B(idxKeyframe),
                preintegrator_));
        // Smoother indices used for marginalization
        newSmootherIndices_[X(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        newSmootherIndices_[V(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        newSmootherIndices_[B(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        // update estimator
        auto stopwatchSmootherStart = std::chrono::high_resolution_clock::now();
        smoother_.update(newSmootherFactors_, newValues_, newSmootherIndices_, factorsToRemove_);
        auto stopwatchSmootherEnd = std::chrono::high_resolution_clock::now();
        auto durationSmoother = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchSmootherEnd - stopwatchSmootherStart).count();
        std::cout << "::: [DEBUG] smoother update took " << durationSmoother << " ms :::" << std::endl;
        summarizeFactors();
        resetNewFactors();
        // Extract estimated state
        w_X_curr_ = gtsam::NavState{
            smoother_.calculateEstimate<gtsam::Pose3>(X(idxKeyframe)),
            smoother_.calculateEstimate<gtsam::Vector3>(V(idxKeyframe))};
        currBias_ = smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B(idxKeyframe));
        w_X_curr_.print();
        std::cout << "Current bias: ";
        currBias_.print();

        // Reset preintegrator
        preintegrator_.resetIntegrationAndSetBias(currBias_);

        // TODO: supplement new clusters from the current keyframe to avoid tracking loss
        // --> only do this when tracking shows reasonable results in the first place ..

        if (idxKeyframe > static_cast<uint32_t>(config_.backend.sliding_window_size))
        {
            uint32_t idxLowerBound = keyframeSubmaps_.begin()->first;
            uint32_t idxUpperbound = idxKeyframe - static_cast<uint32_t>(config_.backend.sliding_window_size);

            std::cout << "::: [DEBUG] marginalizing " << (idxUpperbound - idxLowerBound) << " keyframes :::" << std::endl;
            for (uint32_t idxMargiznalizedKeyframe = idxLowerBound; idxMargiznalizedKeyframe < idxUpperbound; ++idxMargiznalizedKeyframe)
            {
                if (keyframeSubmaps_.find(idxMargiznalizedKeyframe) != keyframeSubmaps_.end())
                {
                    removeKeyframeFromClusters(idxMargiznalizedKeyframe);
                    keyframeSubmaps_.erase(idxMargiznalizedKeyframe);
                    keyframePoses_.erase(idxMargiznalizedKeyframe);
                    keyframeTimestamps_.erase(idxMargiznalizedKeyframe);
#ifndef DISABLEVIZ
                    visualizer.removeSubmap(idxMargiznalizedKeyframe);
                    std::cout << "::: [INFO] marginalized keyframe " << idxMargiznalizedKeyframe << " :::" << std::endl;
                    visualizer.waitForSpacebar();
#endif
                }
            }
        }

        // Update the poses of the keyframe submaps
        for (auto const &[idxKf, _] : keyframeSubmaps_)
        {
            const gtsam::Pose3
                // updated IMU pose in world frame
                world_T_imu = smoother_.calculateEstimate<gtsam::Pose3>(X(idxKf)),
                // updated lidar pose in world frame
                updatedPose = world_T_imu.compose(imu_T_lidar_);
            updateKeyframeSubmapPose(idxKf, updatedPose);
#ifndef DISABLEVIZ
            visualizer.updateSubmap(idxKf, updatedPose.matrix());
#endif
        }
#ifndef DISABLEVIZ
        visualizer.waitForSpacebar();
#endif
    }

    void MappingSystem::resetNewFactors()
    {
        newSmootherFactors_.resize(0);
        newValues_.clear();
        newSmootherIndices_.clear();
        factorsToRemove_.clear();
    }

    uint32_t MappingSystem::createKeyframeSubmap(
        const gtsam::Pose3 &world_T_lidar,
        double keyframeTimestamp,
        std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap)
    {
        // Increment keyframe counter
        const uint32_t idxNewKf = keyframeCounter_++;

        // Transform submap to its estimated pose in the world frame
        ptrKeyframeSubmap->Transform(world_T_lidar.matrix());
        keyframeSubmaps_[idxNewKf] = ptrKeyframeSubmap;
        keyframePoses_[idxNewKf] = std::make_shared<gtsam::Pose3>(world_T_lidar);
        keyframeTimestamps_[idxNewKf] = keyframeTimestamp;

        std::cout << "::: [DEBUG] created keyframe " << idxNewKf << " ("
                  << ptrKeyframeSubmap->points_.size() << " pts) at timestamp "
                  << keyframeTimestamp << " :::" << std::endl;

        // Clear scan buffer
        scanBuffer_.clear();
        // reset scan counter
        scansSinceLastKeyframe_ = 0;
        return idxNewKf;
    }

    gtsam::Pose3 MappingSystem::lastKeyframePose() const
    {
        return *keyframePoses_.rbegin()->second;
    }

    void MappingSystem::updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &w_T_l)
    {
        const gtsam::Pose3 deltaPose = keyframePoses_[keyframeIdx]->between(w_T_l);
        keyframeSubmaps_[keyframeIdx]->Transform(deltaPose.matrix());
        keyframePoses_[keyframeIdx] = std::make_shared<gtsam::Pose3>(w_T_l);
    }

    void MappingSystem::bufferScan(
        const gtsam::Pose3 &scanPoseToLastKeyframe,
        std::shared_ptr<open3d::geometry::PointCloud> pcdScan)
    {
        scanBuffer_.push_back(ScanBuffer{pcdScan, std::make_shared<gtsam::Pose3>(scanPoseToLastKeyframe)});
        scansSinceLastKeyframe_++;
    }

    void MappingSystem::summarizeClusters() const
    {
        std::cout << "::: [DEBUG] Cluster summary :::" << std::endl;
        for (auto const &cluster : clusters_)
        {
            auto const &[clusterId, clusterPoints] = cluster;
            // skip invalid clusters, otherwise it's too much logging going on
            if (!clusterValidity_.at(clusterId))
                continue;
            bool hasParams = clusterPlaneThickness_.find(clusterId) != clusterPlaneThickness_.end() && clusterSigmas_.find(clusterId) != clusterSigmas_.end() && clusterValidity_.find(clusterId) != clusterValidity_.end();
            if (!hasParams)
            {
                std::cout << "\tId: " << clusterId << " size: " << clusterPoints.size() << " (no params)" << std::endl;
                continue;
            }
            std::cout << "\tId: " << clusterId << " size: " << clusterPoints.size() << " thickness: " << clusterPlaneThickness_.at(clusterId)
                      << " sigma: " << clusterSigmas_.at(clusterId)
                      << " valid: " << (clusterValidity_.at(clusterId) ? "true" : "false") << std::endl;
        }
    }

    SlidingWindowStates MappingSystem::getStates() const
    {
        SlidingWindowStates states;
        for (auto const &[idxKf, _] : keyframeSubmaps_)
        {
            try
            {
                const gtsam::Pose3 kfPose = smoother_.calculateEstimate<gtsam::Pose3>(X(idxKf));
                const gtsam::Vector3 kfVel = smoother_.calculateEstimate<gtsam::Vector3>(V(idxKf));
                states[idxKf] = NavStateStamped{gtsam::NavState{kfPose, kfVel}, keyframeTimestamps_.at(idxKf)};
            }
            catch (const std::out_of_range &e)
            {
                std::cerr << "::: [WARNING] could not retrieve state for keyframe " << idxKf << " :::" << std::endl;
            }
        }
        return states;
    }

    std::shared_ptr<open3d::geometry::PointCloud> MappingSystem::getCurrentSubmap() const
    {
        if (keyframeSubmaps_.empty())
        {
            std::cerr << "::: [WARNING] no keyframe submaps available :::" << std::endl;
            return nullptr;
        }
        return keyframeSubmaps_.rbegin()->second;
    }

    std::map<ClusterId, PointCluster> MappingSystem::getCurrentClusters() const
    {
        std::map<ClusterId, PointCluster> currentClusters;
        for (auto clusterIt = clusters_.begin(); clusterIt != clusters_.end(); ++clusterIt)
        {
            auto const &[clusterId, clusterPoints] = *clusterIt;
            if (!clusterValidity_.at(clusterId))
                continue;
            PointCluster clusterRepresentation{clusterCenters_.at(clusterId), clusterNormals_.at(clusterId)};
            currentClusters.emplace(clusterId, clusterRepresentation);
        }
        return currentClusters;
    }

    void MappingSystem::summarizeFactors() const
    {
        const gtsam::NonlinearFactorGraph &factors = smoother_.getFactors();
        size_t numImuFactors{0}, numLidarFactors{0};
        for (size_t factorKey = 0; factorKey < factors.size(); ++factorKey)
        {
            const gtsam::NonlinearFactor::shared_ptr factor = factors[factorKey];
            const auto imuFactor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
            if (imuFactor)
            {
                numImuFactors++;
                continue;
            }
            const auto ptpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
            if (ptpFactor)
            {
                numLidarFactors++;
                continue;
            }
        }
        std::cout << "::: [DEBUG] smoother has " << numImuFactors << " IMU factors, " << numLidarFactors << " LiDAR factors." << std::endl;
    }

} // namespace mapping
