#include <mapping/MappingSystem.hpp>

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <csignal>
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
        : kernel_(gtsam::noiseModel::mEstimator::GemanMcClure::Create(1.0)), // c = 1.0
          systemState_(SystemState::Initializing),
          keyframeCounter_(0),
          tLastImu_(0.0),
          clusterIdCounter_(0),
          config_(config),
          imu_T_lidar_(config.extrinsics.imu_T_lidar.toPose3()),
          lidarTimeOffset_(0.0)
    {
        /**
         * NOTE: avoid using config_ here as much as possible or
         * make it explicit and repeatable (e.g. smoother init)
         * because setConfig could be called after the constructor
         */
        initializeSmoother(config_); // <-- uses config_
        // Initialize IMU preintegration parameters
        auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        // noise values from:
        // https://github.com/RomanStadlhuber/mid360_docker/blob/main/mid360_runner/config/mapping_mid360.yaml#L13
        constexpr double
            // sensor readout noise
            accelerometerNoise = 0.05,
            gyroscopeNoise = 0.005,
            // bias random walk noise
            accelBiasRandomWalk = 0.0005,
            gyroBiasRandomWalk = 0.00005;
        // sensor readout noise
        params->accelerometerCovariance = gtsam::I_3x3 * std::pow(accelerometerNoise, 2);
        params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(gyroscopeNoise, 2);
        // bias random walk noise
        params->biasAccCovariance = gtsam::I_3x3 * std::pow(accelBiasRandomWalk, 2);
        params->biasOmegaCovariance = gtsam::I_3x3 * std::pow(gyroBiasRandomWalk, 2);
        // bias value from:
        // https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/include/use-ikfom.hpp#L35
        params->integrationCovariance = gtsam::I_3x3 * 1e-8;
        const gtsam::Vector6 commonBias{gtsam::Vector6::Ones() * 0.0001};
        gtsam::imuBias::ConstantBias priorImuBias{commonBias};
        preintegrator_ = gtsam::PreintegratedCombinedMeasurements(params, priorImuBias);
    }

    void MappingSystem::setConfig(const MappingConfig &config)
    {
        config_ = config;
        imu_T_lidar_ = config_.extrinsics.imu_T_lidar.toPose3();
        initializeSmoother(config_);
    }

    void MappingSystem::initializeSmoother(const MappingConfig &config)
    {
        // need to reset smoother with new config
        gtsam::ISAM2Params smootherParams;
        smootherParams.optimizationParams = gtsam::ISAM2GaussNewtonParams();
        // for some guidance, see:
        // https://github.com/MIT-SPARK/Kimera-VIO/blob/master/include/kimera-vio/backend/VioBackendParams.h
        smootherParams.relinearizeThreshold = 0.01;  // relinearsize more often for better estimates?
        smootherParams.relinearizeSkip = 1;          // only (check) relinearize after that many update calls
        smootherParams.findUnusedFactorSlots = true; // should be enabled when using smoother
        std::cout << "Initializing smoother with fixed lag of " << config.backend.sliding_window_size << std::endl;
        smoother_ = gtsam::IncrementalFixedLagSmoother(static_cast<double>(config.backend.sliding_window_size), smootherParams);
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
            std::cout << "::: [ERROR] tracking lost. recovery not implemented, shutting down :::" << std::endl;
            std::raise(SIGINT);
            return;
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
        createNewClusters(idxNewKF, /*voxelSize=*/0);
        lastClusterKF_ = idxNewKF;
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
            gyroVariance += (it->second->angular_velocity - gyroBiasMean).squaredNorm();
        }
        accVariance /= (numImuSamples - 1.0);
        gyroVariance /= (numImuSamples - 1.0);
        // preintegrator_.params()->accelerometerCovariance = gtsam::I_3x3 * accVariance;
        // preintegrator_.params()->gyroscopeCovariance = gtsam::I_3x3 * gyroVariance;

        // Set bias (use existing acceleration bias)
        gtsam::imuBias::ConstantBias priorImuBias{preintegrator_.biasHat().accelerometer(), gyroBiasMean};
        preintegrator_.resetIntegrationAndSetBias(priorImuBias);

        // Construct Navigation State prior (mean & covariance)
        resetNewFactors();
        gtsam::Vector6 priorPoseSigma;
        priorPoseSigma << 0.0175, 0.0175, 0.0873, 0.1, 0.1, 0.1; // rpy xyz

        // Pose prior should be certain
        newSmootherFactors_.addPrior(X(idxNewKF), w_T_i0, gtsam::noiseModel::Diagonal::Sigmas(priorPoseSigma));
        newSmootherFactors_.addPrior(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Isotropic::Sigma(3, 1e-2));
        // Bias prior, very noisy
        const gtsam::noiseModel::Diagonal::shared_ptr biasPriorNoise =
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.01, 0.01, 0.01, // gyro bias (rad/s)
                 0.1, 0.1, 0.1                         // accel bias (m/s^2)
                 )
                    .finished());
        newSmootherFactors_.addPrior(
            B(idxNewKF),
            priorImuBias,
            biasPriorNoise);
        // initial estimates
        newValues_.insert(X(idxNewKF), w_T_i0);
        newValues_.insert(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()));
        newValues_.insert(B(idxNewKF), priorImuBias);
        w_X_curr_ = gtsam::NavState(w_T_i0, gtsam::Vector3::Zero());
        w_X_preint_ = w_X_curr_;
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

        const gtsam::NavState w_X_propagated = preintegrator_.predict(w_X_preint_, currBias_);
        return w_X_propagated;
    }

    void MappingSystem::undistortScans()
    {
        // Iterator to the newest lidar scan that can still be processed (older than latest imu timestamp)
        std::unique_lock<std::mutex> lockLidarBuffer(mtxLidarBuffer_);

        // Undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation
        // Delta pose to last submap at preintegrated state / last IMU time
        gtsam::Pose3 kf_T_prop = lastKeyframePose().between(w_X_curr_.pose().compose(imu_T_lidar_));

        const double tLastKeyframe = keyframeTimestamps_.rbegin()->second;
        // time delta between last keyframe and new preintegration
        const double dtPropToKeyframe = tLastImu_ - tLastKeyframe;

        // Pose delta of this scan to last keyframe
        // Compute pose delta to last submap at current scan time (extrapolate with constant velocity)
        // Linear and angular velocity are obtained from pose delta over preintegration time
        const gtsam::Vector3 angVel = gtsam::Rot3::Logmap(kf_T_prop.rotation()) / dtPropToKeyframe;
        const gtsam::Vector3 linVel = kf_T_prop.translation() / dtPropToKeyframe;

        double tLastScan = tLastKeyframe;
        // Undistort between individual scans
        gtsam::Pose3 deltaPoseLastScanToKeyframe = gtsam::Pose3::Identity();
        for (auto it = lidarBuffer_.begin(); it != lidarBuffer_.end(); ++it)
        {
            const auto [tScan, scan] = *it;
            const double dtScanToKeyframe = tScan - tLastKeyframe;

            // Apply scan to keyframe delta time to get pose delta of the scan w.r.t. the last keyframe
            gtsam::Rot3 kf_R_scan = gtsam::Rot3::Expmap(angVel * dtScanToKeyframe);
            gtsam::Point3 kf_P_scan = linVel * dtScanToKeyframe;
            gtsam::Pose3 kf_T_scan{kf_R_scan, kf_P_scan};

            // Pose & time delta from this to last scan
            gtsam::Pose3 lastScan_T_currScan = deltaPoseLastScanToKeyframe.between(kf_T_scan);
            const double dtScanToLastScan = tScan - tLastScan;
            // velocity between subsequent scans (deltaPose / deltaTime)
            const gtsam::Vector3
                scanAngVel = gtsam::Rot3::Logmap(lastScan_T_currScan.rotation()) / dtScanToLastScan,
                scanLinVel = lastScan_T_currScan.translation() / dtScanToLastScan;
            // Undistort current scan
            for (std::size_t i = 0; i < scan->points.size(); ++i)
            {
                const Eigen::Vector3d pt = scan->points[i];
                const double dt = scan->offset_times[i];

                // Presumed rotation and translation that the point should have undergone during scan time
                gtsam::Rot3 scan_R_pt = gtsam::Rot3::Expmap(scanAngVel * dt);
                gtsam::Point3 scan_P_pt = scanLinVel * dt;
                gtsam::Pose3 scan_T_pt{scan_R_pt, scan_P_pt};

                // Undistort point with inverse to correct motion distortion
                const Eigen::Vector3d ptUndistorted = scan_T_pt.transformFrom(pt);
                scan->points[i] = ptUndistorted;
            }
            /**
             * Convert scan to pointcloud, voxelize and buffer with relative pose to last keyframe
             * NOTE: the undistorted scan pointcloud is not yet transformed as this
             */
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, config_.point_filter.min_distance, config_.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(config_.lidar_frontend.voxel_size);

            // Buffer undistorted scan
            bufferScan(kf_T_scan, ptrPcdScanVoxelized);
            // update pose and time reference for computing delta between scans in next iteration
            deltaPoseLastScanToKeyframe = kf_T_scan;
            tLastScan = tScan;
        }

        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // Erase processed raw scan data and release buffer lock
        lidarBuffer_.clear();
        lockLidarBuffer.unlock();
    }

    std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, double> MappingSystem::planeFitSVD(
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

        double planeThickness = 0.0;
        if (isValid)
        {
            // Compute plane thickness as mean squared point-to-plane distance
            for (size_t i = 0; i < numPoints; ++i)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(planePoints.row(i)));
                planeThickness += std::pow(pointToPlaneDist, 2.0);
            }
            planeThickness /= static_cast<double>(numPoints);
        }

        return {isValid, planeNormal, planeCenter, planePoints, planeThickness};
    }

    bool MappingSystem::trackScanPointsToClusters(const uint32_t &idxKeyframe)
    {
        // Search for same-plane point clusters among all keyframes
        std::vector<int> knnIndices(config_.lidar_frontend.knn_neighbors);
        knnIndices.reserve(config_.lidar_frontend.knn_neighbors);
        std::vector<double> knnDists(config_.lidar_frontend.knn_neighbors);
        knnDists.reserve(config_.lidar_frontend.knn_neighbors);
        std::size_t validTracks = 0, numValidClusters = 0;
        // TODO: remove stopwatch after slowdown was diagnosed
        auto stopwatchKNNStart = std::chrono::high_resolution_clock::now();
        // KD-Tree of the current submap, used for cluster tracking
        const open3d::geometry::KDTreeFlann kdTree{*keyframeSubmaps_[idxKeyframe]};
        // project each cluster point onto the current keyframe and try to find the 5 nearest neighbors
        for (auto const &cluster : clusters_)
        {
            auto const &[clusterId, clusterPointIdxs] = cluster;
            const ClusterState clusterState = clusterStates_.at(clusterId);
            // --- ignore pruned clusters ---
            if (clusterState == ClusterState::Pruned)
                continue;
            // --- tracking: KNN search & SVD plane fit ---
            auto const &[idxClusterKF, idxSubmapPt] = *clusterPointIdxs.rbegin(); // get the oldest point in the cluster
            const Eigen::Vector3d &world_clusterPt = keyframeSubmaps_[idxClusterKF]->points_[idxSubmapPt];
            const int knnFound = kdTree.SearchKNN(
                world_clusterPt,
                config_.lidar_frontend.knn_neighbors,
                knnIndices,
                knnDists);
            // collect KNN points and fit a plane
            std::vector<Eigen::Vector3d> knnPoints;
            knnPoints.reserve(knnFound);
            for (int i = 0; i < knnFound; ++i)
                knnPoints.push_back(keyframeSubmaps_[idxKeyframe]->points_[knnIndices[i]]);
            const auto [validPlaneTrack, planeTrackNormal, knnCenter, knnPointsMat, planeTrackThickness] = planeFitSVD(knnPoints);
            // --- tracking failed (KNN plane fit invalid): update cluster center & normal, keep thickness ---
            if (!validPlaneTrack || planeTrackThickness > config_.lidar_frontend.clustering.max_plane_thickness)
            {
                if (clusterState == ClusterState::Premature) // don't update premature clusters
                    continue;
                clusterStates_[clusterId] = ClusterState::Idle;
                updateClusterParameters(clusterId, false); // update cluster, keep thickness (no new KF association)
                continue;
            }
            static constexpr size_t IDX_KNN_POINT = 1; // use second-nearest neighbor for tracking to increase plane stability
            // -- 6-sigma test for new plane point with current cluster center & plane normal ---
            if (clusterState != ClusterState::Premature)
            {
                const Eigen::Vector3d &knnPoint = keyframeSubmaps_.at(idxKeyframe)->points_.at(knnIndices[IDX_KNN_POINT]);
                const double pointToPlaneDist = std::abs(clusterNormals_.at(clusterId)->dot(knnPoint - *clusterCenters_.at(clusterId)));
                if (pointToPlaneDist >= 3.0 * clusterSigmas_.at(clusterId))
                {
                    clusterStates_[clusterId] = ClusterState::Idle;
                    updateClusterParameters(clusterId, false); // update cluster, keep thickness (no new KF association)
                    continue;
                }
            }
            // --- tracking valid: add point to cluster ---
            // associate the second-nearest point with the cluster to increase stability
            addPointToCluster(clusterId, {idxKeyframe, knnIndices[IDX_KNN_POINT]}, planeTrackThickness);
            validTracks++;
            knnIndices.clear();
            knnDists.clear();
            const bool clusterTooSmall = clusterPointIdxs.size() < config_.lidar_frontend.clustering.min_points;
            if (clusterTooSmall || clusterState == ClusterState::Pruned)
                continue; // skip
            // collect all points associated with this cluster
            std::vector<Eigen::Vector3d> clusterPoints;
            clusterPoints.reserve(clusterPointIdxs.size());
            for (auto const &pointIdxPair : clusterPointIdxs)
            {
                const auto &[idxSubmap, idxPoint] = pointIdxPair;
                clusterPoints.push_back(keyframeSubmaps_[idxSubmap]->points_[idxPoint]);
            }
            const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat, planeThickness] = planeFitSVD(clusterPoints);
            // --- new plane normal consistency check ---
            static constexpr double NORMAL_CONSISTENCY_THRESHOLD = 0.9;
            if (
                clusterState == ClusterState::Premature // for premature clusters, update immediately
                // otherwise perform consistency check
                || (planeValid && std::abs(clusterNormals_.at(clusterId)->dot(planeNormal)) > NORMAL_CONSISTENCY_THRESHOLD))
            {
                clusterStates_[clusterId] = ClusterState::Tracked;
                // Note: internally uses thickness history to update covariance
                updateClusterParameters(clusterId, planeNormal, clusterCenter);
                numValidClusters++;
            }
            else
            {
                // conistency check failed -> mark cluster Idle, remove added pt and recompute parameters from old old associations
                if (clusterState == ClusterState::Premature) // must not go from premature to idle
                    continue;
                clusterStates_[clusterId] = ClusterState::Idle; // idle - no valid track in newest KF
                removePointFromCluster(clusterId, idxKeyframe); // remove latest association
                updateClusterParameters(clusterId, false);      // update location, thickness shouldn't change (no new KF association)
                continue;
            }
        }
        if (validTracks == 0) // abort if the latest frame could not be tracked
        {
            return false;
        }
        auto stopwatchKNNEnd = std::chrono::high_resolution_clock::now();
        auto durationKNN = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchKNNEnd - stopwatchKNNStart).count();
        std::cout << "::: [DEBUG] KNN search and cluster association took " << durationKNN << " ms :::" << std::endl;
        std::cout << "::: [INFO] keyframe " << idxKeyframe << " had " << validTracks << " tracks and " << numValidClusters << " valid clusters :::" << std::endl;
        return idxKeyframe < config_.lidar_frontend.clustering.min_points ? true : numValidClusters > 0;
    }

    void MappingSystem::createNewClusters(const uint32_t &idxKeyframe, double voxelSize)
    {
        std::size_t numCreated = 0;

        if (voxelSize <= 0.01) // use all points for creating new clusters
        {
            for (std::size_t idxPoint = 0; idxPoint < keyframeSubmaps_.at(idxKeyframe)->points_.size(); idxPoint++)
            {
                std::map<uint32_t, std::size_t> newCluster({{idxKeyframe, idxPoint}});
                auto const clusterId = clusterIdCounter_++;
                clusters_.emplace(clusterId, newCluster);
                clusterStates_.emplace(clusterId, ClusterState::Premature);
                clusterCenters_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(keyframeSubmaps_[idxKeyframe]->points_[idxPoint]));
                clusterNormals_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero())); // safe guard, will yield zero-residuals
                numCreated++;
            }
        }
        else // use downsampling (but doesn't store the pcd) to create new clusters
        {
            const std::shared_ptr<open3d::geometry::PointCloud> &submap = keyframeSubmaps_.at(idxKeyframe);
            // according to source code: (output, cubic_id, original_indices)
            auto const [_, __, voxelizedIndices] = submap->VoxelDownSampleAndTrace(
                voxelSize,
                submap->GetMinBound(),
                submap->GetMaxBound());

            for (const auto &idxsVoxelPts : voxelizedIndices)
            {
                std::size_t idxPoint = idxsVoxelPts.front(); // use first point in voxel for cluster creation
                std::map<uint32_t, std::size_t> newCluster({{idxKeyframe, idxPoint}});
                auto const clusterId = clusterIdCounter_++;
                clusters_.emplace(clusterId, newCluster);
                clusterStates_.emplace(clusterId, ClusterState::Premature);
                clusterCenters_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(keyframeSubmaps_[idxKeyframe]->points_[idxPoint]));
                clusterNormals_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero())); // safe guard, will yield zero-residuals
                numCreated++;
            }
        }
        std::cout << "::: [INFO] Created " << numCreated << " new clusters from keyframe " << idxKeyframe << " :::" << std::endl;
    }

    void MappingSystem::addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness)
    {
        auto const &[idxSubmap, idxPoint] = pointIdx; // destructure to make it clearer what's going on
        clusters_[clusterId][idxSubmap] = idxPoint;
        clusterPlaneThicknessHistory_[clusterId].push_back(planeThickness);
    }

    void MappingSystem::removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory)
    {
        { // remove keyframe point from cluster
            auto it = clusters_[clusterId].find(idxKeyframe);
            if (it != clusters_[clusterId].end())
                clusters_[clusterId].erase(it);
        }
        { // remove thickness entry
            auto it = clusterPlaneThicknessHistory_.find(clusterId);
            // 2nd: vector<double> cluster thickness history (last entry should be latest keyframe)
            if (it != clusterPlaneThicknessHistory_.end() && !it->second.empty())
            {
                if (firstInHistory)
                    it->second.erase(it->second.begin());
                else
                    it->second.pop_back();
            }
        }
    }

    void MappingSystem::updateClusterParameters(const ClusterId &clusterId, bool recalcPlaneThickness)
    {
        std::vector<Eigen::Vector3d> clusterPoints;
        clusterPoints.reserve(clusters_[clusterId].size());
        for (auto const &[idxSubmap, idxPoint] : clusters_.at(clusterId))
        {
            clusterPoints.push_back(keyframeSubmaps_[idxSubmap]->points_[idxPoint]);
        }
        const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat, planeThickness] = planeFitSVD(clusterPoints);
        *clusterCenters_[clusterId] = clusterCenter;
        *clusterNormals_[clusterId] = planeNormal;
        // explicitly recalculate plane thickness when a point was added or removed
        if (recalcPlaneThickness)
        {
            double planeThicknessCovariance = 0.0;
            for (const double &thickness : clusterPlaneThicknessHistory_[clusterId])
                planeThicknessCovariance += std::pow(thickness, 2.0);
            planeThicknessCovariance /= static_cast<double>(clusterPlaneThicknessHistory_[clusterId].size());
            clusterPlaneThickness_[clusterId] = planeThicknessCovariance;
            clusterSigmas_[clusterId] = std::pow(0.5 * planeThicknessCovariance, 0.25);
        }
    }

    void MappingSystem::updateClusterParameters(
        const ClusterId &clusterId,
        const Eigen::Vector3d &planeNormal,
        const Eigen::Vector3d &clusterCenter)
    {
        *clusterCenters_[clusterId] = clusterCenter;
        *clusterNormals_[clusterId] = planeNormal;
        double planeThicknessCovariance = 0.0;
        for (const double &thickness : clusterPlaneThicknessHistory_[clusterId])
            planeThicknessCovariance += std::pow(thickness, 2.0);
        planeThicknessCovariance /= static_cast<double>(clusterPlaneThicknessHistory_[clusterId].size());
        clusterPlaneThickness_[clusterId] = planeThicknessCovariance;
        clusterSigmas_[clusterId] = std::pow(0.5 * planeThicknessCovariance, 0.25);
    }

    void MappingSystem::removeKeyframeFromClusters(const uint32_t &idxKeyframe)
    {
        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            auto itPoint = clusterPoints.find(idxKeyframe);
            if (itPoint != clusterPoints.end())
            {
                removePointFromCluster(clusterId, idxKeyframe, /*firstInHistory=*/true); // remove point and thickness history entry
                if (clusterPoints.size() < 3)
                { // TODO: use min-points size or 3?
                    // NOTE: Do NOT call factor->remove() here, since the smoother won't re-key factors
                    clusterStates_[clusterId] = ClusterState::Pruned;
                    continue;
                }
                if (clusterStates_[clusterId] == ClusterState::Idle)
                {
                    // shift cluster associations, means that the entire factor needs to be replaced
                    // to reflect the removed keyframe association
                    clusterStates_[clusterId] = ClusterState::ShiftedIdle;
                }
            }
        }
    }

    void MappingSystem::pruneClusters(const uint32_t &idxKeyframe)
    {
        if (idxKeyframe < static_cast<size_t>(config_.backend.sliding_window_size))
            return;
        std::set<ClusterId> clustersToErase;
        for (const auto &[clusterId, clusterPoints] : clusters_)
            // erase if cluster has enough points but is invalid
            if (clusterStates_.at(clusterId) == ClusterState::Pruned)
                clustersToErase.insert(clusterId);

        for (const auto &clusterId : clustersToErase)
        {
            clusters_.erase(clusterId);
            clusterStates_.erase(clusterId);
            clusterCenters_.erase(clusterId);
            clusterNormals_.erase(clusterId);
            clusterPlaneThickness_.erase(clusterId);
            clusterSigmas_.erase(clusterId);
            clusterPlaneThicknessHistory_.erase(clusterId);
            clusterFactors_.erase(clusterId);
        }
        std::cout << "::: [INFO] Pruned " << clustersToErase.size() << " clusters, "
                  << clusters_.size() << " clusters remain :::" << std::endl;
    }
    void MappingSystem::createAndUpdateFactors(const uint32_t &idxKeyframe)
    {
        std::size_t numFactorsAdded{0}, numFactorsUpdated{0}, numFactorsRemoved{0};
        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            const ClusterState clusterState = clusterStates_.at(clusterId);
            // skip premature clusters, but pruned clusters need to be handled explicitly
            if (clusterState == ClusterState::Premature)
                continue;
            auto existingFactorIt = clusterFactors_.find(clusterId);
            // decide what to do based on the cluster state
            switch (clusterState)
            {
            case ClusterState::Tracked:
            {
                // cluster parameters from new track
                const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
                const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
                const double adaptiveSigma = clusterSigmas_[clusterId];
                if (existingFactorIt == clusterFactors_.end()) // factor does not exist, create
                {

                    // 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
                    gtsam::KeyVector keys;
                    keys.reserve(clusterPoints.size());
                    // 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
                    std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                    for (auto const &[idxKeyframe, idxPoint] : clusterPoints)
                    {
                        const gtsam::Key key = X(idxKeyframe);
                        keys.push_back(key);
                        // scan points are passed to factor in world frame
                        lidar_points[key] = keyframePoses_[idxKeyframe]->transformTo(keyframeSubmaps_[idxKeyframe]->points_[idxPoint]);
                    }
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto factor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        imu_T_lidar_,
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId);
                    newSmootherFactors_.add(factor);
                    // factor->print();
                    clusterFactors_[clusterId] = factor;
                    numFactorsAdded++;
                }
                else // cluster factor exists, remove old and readd with updated keys
                {
                    // 1: Remove old factor from smoother (if still present)
                    const gtsam::NonlinearFactorGraph &smootherFactors = smoother_.getFactors();
                    for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                    {
                        const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                        const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                        if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                        {
                            existingPtpFactor->markInvalid();
                            factorsToRemove_.push_back(gtsam::Key{factorKey});
                            numFactorsRemoved++;
                            break;
                        }
                    }
                    // 2: Create new factor from ALL current clusterPoints
                    gtsam::KeyVector keys;
                    keys.reserve(clusterPoints.size());
                    std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                    for (auto const &[kfIdx, ptIdx] : clusterPoints)
                    {
                        const gtsam::Key key = X(kfIdx);
                        keys.push_back(key);
                        lidar_points[key] = keyframePoses_[kfIdx]->transformTo(keyframeSubmaps_[kfIdx]->points_[ptIdx]);
                    }
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        imu_T_lidar_,
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId);
                    newSmootherFactors_.add(newFactor);
                    clusterFactors_[clusterId] = newFactor;
                    // std::cout << "::: [DEBUG] replaced factor for cluster " << clusterId << " with " << keys.size() << " keys :::" << std::endl;
                    // newFactor->print();
                    numFactorsUpdated++;
                }
            }
            break;
            case ClusterState::Idle: // key associations did not change but new state estimates cause updated plane parameters
            {
                if (existingFactorIt != clusterFactors_.end())
                {
                    boost::shared_ptr<PointToPlaneFactor> factor = existingFactorIt->second;
                    const double adaptiveSigma = clusterSigmas_[clusterId];
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    factor->updatePlaneParameters(clusterNormals_[clusterId], clusterCenters_[clusterId], robustNoise);
                }
            }
            break;
            case ClusterState::ShiftedIdle: // key associations changed but cluster is idle, so plane parameters are not updated
            {
                // TODO: is the same as "tracked" case, just no association to the latest keyframe?
                if (existingFactorIt == clusterFactors_.end())
                {
                    std::cout << "::: [WARN] attempting to shift factor keys of idle cluster, but factor does not exist :::" << std::endl;
                    continue; // should not happen, but safe guard
                }
                const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
                const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
                const double adaptiveSigma = clusterSigmas_[clusterId];
                // 1: Remove old factor from smoother (if still present)
                const gtsam::NonlinearFactorGraph &smootherFactors = smoother_.getFactors();
                for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                {
                    const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                    const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                    if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                    {
                        existingPtpFactor->markInvalid();
                        factorsToRemove_.push_back(gtsam::Key{factorKey});
                        numFactorsRemoved++;
                        break;
                    }
                }
                // 2: Create new factor from ALL current clusterPoints
                gtsam::KeyVector keys;
                keys.reserve(clusterPoints.size());
                std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                for (auto const &[kfIdx, ptIdx] : clusterPoints)
                {
                    const gtsam::Key key = X(kfIdx);
                    keys.push_back(key);
                    lidar_points[key] = keyframePoses_[kfIdx]->transformTo(keyframeSubmaps_[kfIdx]->points_[ptIdx]);
                }
                const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                    keys,
                    imu_T_lidar_,
                    lidar_points,
                    clusterNormal,
                    clusterCenter,
                    robustNoise,
                    clusterId);
                newSmootherFactors_.add(newFactor);
                clusterFactors_[clusterId] = newFactor;
                numFactorsUpdated++;
                clusterStates_[clusterId] = ClusterState::Idle; // after shifting, cluster goes back to idle state
            }
            break;
            case ClusterState::Pruned: // factor must be removed from the smoother
            {
                const gtsam::NonlinearFactorGraph &smootherFactors = smoother_.getFactors();
                for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                {
                    const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                    const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                    if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                    {
                        existingPtpFactor->markInvalid(); // mark factor as invalid to avoid further optimization
                        // mark existing factor for removal
                        factorsToRemove_.push_back(gtsam::Key{factorKey});
                        numFactorsRemoved++;
                        break;
                    }
                }
            }
            break;
            default:
                // should not reach here due to earlier checks
                continue;
            }
        }
        std::cout << "::: [INFO] adding " << numFactorsAdded
                  << " LiDAR factors, updating " << numFactorsUpdated
                  << " removing " << numFactorsRemoved << " :::" << std::endl;
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

        /**
         * NOTE: marginalization is done BEFORE tracking so that when smoother_.update() is called,
         * the to-be-marginalized keyframe's variables are already no longer associated with any factors
         */
        marginalizeKeyframesOutsideSlidingWindow(idxKeyframe); // remove to-be-marginalized keyframe associations
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
        // summarizeClusters();
        auto stopwatchFactorsStart = std::chrono::high_resolution_clock::now();
        // NOTE: will internally update factorsToRemove to drop outdated smart factors
        // the outdated factors will be replaced by extended ones with additional tracks
        createAndUpdateFactors(idxKeyframe);
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
        const double
            tSmootherLast{static_cast<double>(idxKeyframe - 1)},
            tSmootherCurr{static_cast<double>(idxKeyframe)};
        newSmootherIndices_[X(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices_[V(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices_[B(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices_[X(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices_[V(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices_[B(idxKeyframe)] = tSmootherCurr;
        // update estimator
        auto stopwatchSmootherStart = std::chrono::high_resolution_clock::now();
        smoother_.update(newSmootherFactors_, newValues_, newSmootherIndices_, factorsToRemove_);
        /**
         * NOTE: iSAM2 update performs only one GN step,
         * multiple updates to assure convergence, see also
         * - https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/IncrementalFixedLagSmoother.cpp#L128
         * - https://groups.google.com/g/gtsam-users/c/Cz2RoY3dN14/m/3Ka6clsdBgAJ
         */
        for (std::size_t updateIters = 1; updateIters < config_.backend.solver_iterations; updateIters++)
            smoother_.update();
        auto stopwatchSmootherEnd = std::chrono::high_resolution_clock::now();
        auto durationSmoother = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchSmootherEnd - stopwatchSmootherStart).count();
        std::cout << "::: [DEBUG] smoother update took " << durationSmoother << " ms :::" << std::endl;
        summarizeFactors();
        resetNewFactors();
        smootherEstimate_ = smoother_.calculateEstimate();
        // Extract estimated state
        w_X_curr_ = gtsam::NavState{
            smootherEstimate_.at(X(idxKeyframe)).cast<gtsam::Pose3>(),
            smootherEstimate_.at(V(idxKeyframe)).cast<gtsam::Vector3>()};
        currBias_ = smootherEstimate_.at(B(idxKeyframe)).cast<gtsam::imuBias::ConstantBias>();
        std::cout << "Current bias: ";
        currBias_.print();
        auto marginals = smoother_.marginalCovariance(B(idxKeyframe));
        std::cout << "::: [DEBUG] bias marginal trace " << marginals.trace() << " :::" << std::endl;
        // Reset preintegrator
        w_X_preint_ = w_X_curr_;
        preintegrator_.resetIntegrationAndSetBias(currBias_);
        // supplement new clusters from keyframe points
        if (idxKeyframe - lastClusterKF_ >= 6)
        {
            createNewClusters(idxKeyframe - 1, /*voxelSize=*/config_.lidar_frontend.clustering.sampling_voxel_size);
            lastClusterKF_ = idxKeyframe;
        }
        // Update the poses of the keyframe submaps
        std::cout << "::: [INFO] updating keyframe submap poses for ";
        for (auto const &[idxKf, _] : keyframeSubmaps_)
        {
            std::cout << idxKf << " ";
        }
        std::cout << " :::" << std::endl;
        for (auto const &[idxKf, _] : keyframeSubmaps_)
        {
            const gtsam::Pose3
                // updated IMU pose in world frame
                world_T_imu = smootherEstimate_.at(X(idxKf)).cast<gtsam::Pose3>(),
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

    void MappingSystem::marginalizeKeyframesOutsideSlidingWindow(const uint32_t &idxKeyframe)
    {
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
                    if (collectMarginalizedSubmaps_)
                        marginalizedSubmaps_.push_back(keyframeSubmaps_[idxMargiznalizedKeyframe]);
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
        // let D be the delta that needs to be left-applied via Open3D's transform
        // D * T1 = T2 | (...) * T1^-1
        // => D = T2 * T1^-1
        const gtsam::Pose3 deltaPose = w_T_l.compose(keyframePoses_[keyframeIdx]->inverse());
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
            if (!isClusterValid(clusterId)) // skip invalid clusters, otherwise it's too much logging going on
                continue;
            std::string stateStr;
            switch (clusterStates_.at(clusterId))
            {
            case ClusterState::Tracked:
                stateStr = "tracked";
                break;
            case ClusterState::Idle:
                stateStr = "idle";
                break;
            default: // safeguard (shoud not happen)
                stateStr = "premature or marked for removal";
                break;
            }
            std::cout << "\tId: " << clusterId << " size: " << clusterPoints.size() << " thickness: " << clusterPlaneThickness_.at(clusterId)
                      << " sigma: " << clusterSigmas_.at(clusterId)
                      << " state: " << stateStr << std::endl;
        }
    }

    SlidingWindowStates MappingSystem::getStates() const
    {
        SlidingWindowStates states;
        if (smootherEstimate_.empty())
        {
            return states;
        }
        std::cout << ":::" << std::endl;
        for (auto const &[idxKf, _] : keyframeSubmaps_)
        {
            try
            {
                const gtsam::Pose3 kfPose = smootherEstimate_.at(X(idxKf)).cast<gtsam::Pose3>();
                const gtsam::Vector3 kfVel = smootherEstimate_.at(V(idxKf)).cast<gtsam::Vector3>();
                states[idxKf] = NavStateStamped{gtsam::NavState{kfPose, kfVel}, keyframeTimestamps_.at(idxKf)};
            }
            catch (const gtsam::ValuesKeyDoesNotExist &e)
            {
                std::cerr << "::: [WARNING] keyframe " << idxKf << " not found in smoother estimate :::" << std::endl;
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

    uint32_t MappingSystem::getKeyframeCount() const
    {
        return keyframeCounter_;
    }

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> MappingSystem::getMarginalizedSubmaps()
    {
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> submaps;
        std::swap(submaps, marginalizedSubmaps_);
        return submaps;
    }

    void MappingSystem::setCollectMarginalizedSubmaps(bool enable)
    {
        collectMarginalizedSubmaps_ = enable;
    }

    std::map<ClusterId, PointCluster> MappingSystem::getCurrentClusters() const
    {
        std::map<ClusterId, PointCluster> currentClusters;
        for (auto clusterIt = clusters_.begin(); clusterIt != clusters_.end(); ++clusterIt)
        {
            auto const &[clusterId, clusterPoints] = *clusterIt;
            if (!isClusterValid(clusterId))
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
        /* std::cout << "::: [DEBUG] keys currently in the smoother :::" << std::endl;
        smoother_.getFactors().keys().print("  ");
        std::cout << "\n"; */
    }

} // namespace mapping
