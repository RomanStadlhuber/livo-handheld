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

    bool MappingSystem::trackScanPointsToClusters(const uint32_t &idxKeyframe)
    {
        clusters_.clear();

        // Search for same-plane point clusters among all keyframes
        const std::shared_ptr<open3d::geometry::PointCloud> pcdQuery = keyframeSubmaps_[idxKeyframe];
        std::vector<int> knnIndices(config_.lidar_frontend.knn_neighbors);
        std::vector<double> knnDists(config_.lidar_frontend.knn_neighbors);

        // TODO: remove stopwatch after slowdown was diagnosed
        auto stopwatchKNNStart = std::chrono::high_resolution_clock::now();
        // Find nearest neighbors between the new submap and all other keyframe submaps
        for (auto itOtherSubmap = keyframeSubmaps_.begin(); itOtherSubmap != keyframeSubmaps_.end(); ++itOtherSubmap)
        {
            // Skip self
            const uint32_t idxOtherSubmap = itOtherSubmap->first;
            if (idxOtherSubmap == idxKeyframe)
                continue;

            const int numQueryPts = pcdQuery->points_.size();
            for (int idxPt = 0; idxPt < numQueryPts; ++idxPt)
            {
                knnIndices.clear();
                knnDists.clear();
                const int knnFound = submapKDTrees_[idxOtherSubmap]->SearchHybrid(
                    pcdQuery->points_[idxPt], config_.lidar_frontend.knn_radius, config_.lidar_frontend.knn_neighbors, knnIndices, knnDists);

                if (knnFound > 4)
                {
                    ClusterId clusterId = INVALID_CLUSTER_ID;
                    ClusterId tempNewCluster = clusterIdCounter_ + 1;
                    // KNN associations that track the same cluster (redundant id -> target id)
                    std::vector<ClusterId> duplicateTracks;

                    // Identify or assign cluster to every NN returned point
                    for (int i = 0; i < knnFound; ++i)
                    {
                        SubmapIdxPointIdx idxOther{idxOtherSubmap, knnIndices[i]};
                        if (clusterTracks_.find(idxOther) == clusterTracks_.end()) // point is not tracked as part of a cluster yet ..
                        {
                            clusterTracks_[idxOther] = tempNewCluster; // .. assign a temp cluster id until another cluster is found
                        }
                        else
                        {
                            if (clusterId == INVALID_CLUSTER_ID) // no cluster has been assigned to this point yet -> pick the first other
                            {                                    // merge with existing cluster from other point if that cluster is not already full
                                if (clusters_.size() == 0 || clusters_[clusterTracks_[idxOther]].size() < config_.lidar_frontend.clustering.max_points)
                                {
                                    clusterId = clusterTracks_[idxOther];
                                    // a temp cluster MIGHT have been assigned to one of the previous NN points, need to record it for merging
                                    duplicateTracks.push_back(tempNewCluster);
                                }
                                // if there is not enough space to merge into an existing cluster, create a new one
                                else
                                {
                                    clusterId = tempNewCluster;
                                }
                            }
                            // the other point has been assigned to cluster already -> record for merging
                            else if (clusters_.size() == 0 || clusters_[clusterTracks_[idxOther]].size() < config_.lidar_frontend.clustering.max_points)
                            {
                                duplicateTracks.push_back(clusterTracks_[idxOther]);
                            }
                        }

                        // If a new cluster was identified, increment the counter
                        if (clusterId == INVALID_CLUSTER_ID)
                            clusterIdCounter_ = clusterId = tempNewCluster;
                        // or, if this point belongs to an existing cluster
                        else if (duplicateTracks.size() > 0)
                            // assign the common cluster-id to all tracks of duplicate cluster
                            for (const ClusterId &duplicateId : duplicateTracks)
                                for (auto &entry : clusterTracks_)
                                    if (entry.second == duplicateId)
                                        entry.second = clusterId;

                        // Add the current point to the identified cluster
                        SubmapIdxPointIdx idxCurrent{idxKeyframe, idxPt};
                        clusterTracks_[idxCurrent] = clusterId;
                    }
                }
            }
        }
        auto stopwatchKNNEnd = std::chrono::high_resolution_clock::now();
        auto durationKNN = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchKNNEnd - stopwatchKNNStart).count();
        std::cout << "::: [DEBUG] KNN search and cluster association took " << durationKNN << " ms :::" << std::endl;

        // TODO: remove stopwatch after slowdown was diagnosed
        auto stopwatchClusteringStart = std::chrono::high_resolution_clock::now();
        bool keyframeHasTracks = false; // indicates whether the current keyframe is still being tracked
        // Rebuild clusters from tracks (cluster IDs have already been merged in clusterTracks_ above)
        for (auto itTrack = clusterTracks_.begin(); itTrack != clusterTracks_.end(); ++itTrack)
        {
            auto const &[idxPoint, clusterId] = *itTrack;
            if (clusters_.find(clusterId) == clusters_.end())
            {
                clusters_[clusterId] = std::vector<ClusterTracks::iterator>();
                clusters_[clusterId].reserve(config_.lidar_frontend.knn_neighbors);
                clusters_[clusterId].push_back(itTrack);
            }
            else
                clusters_[clusterId].push_back(itTrack);
            if (idxPoint.first == idxKeyframe)
                keyframeHasTracks = true;
        }
        if (!keyframeHasTracks)
        {
            std::cout << "::: [ERROR] lost tracking at keyframe " << idxKeyframe << " :::" << std::endl;
            return keyframeHasTracks;
        }
        // reset because we might remove current tracks as outliers
        keyframeHasTracks = false;

        // Remove clusters that are too small
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end();)
        {
            auto const &[clusterId, clusterPoints] = *itCluster;
            if (clusterPoints.size() < config_.lidar_frontend.clustering.min_points || clusterPoints.size() > config_.lidar_frontend.clustering.max_points * 2)
                itCluster = eraseClusterAndTracks(itCluster);
            else
                ++itCluster;
        }

        // Outlier rejection - remove clusters that aren't planes
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end();)
        {
            auto const &[clusterId, clusterPoints] = *itCluster;
            size_t numHistoricPoints = 0;
            // Compute centered point locations for plane fitting
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const ClusterTracks::iterator &itTrack : itCluster->second)
            {
                auto const &[idxSubmap, pointIdx] = itTrack->first;
                if (idxSubmap == idxKeyframe)
                    continue;
                centroid += keyframeSubmaps_[idxSubmap]->points_[pointIdx];
                numHistoricPoints++;
            }

            if (numHistoricPoints < 4)
            {
                itCluster = eraseClusterAndTracks(itCluster);
                continue;
            }

            const double n = static_cast<double>(numHistoricPoints);
            centroid /= n;
            Eigen::MatrixXd A(numHistoricPoints, 3);
            size_t idxClusterPt = 0;
            for (const ClusterTracks::iterator &itTrack : itCluster->second)
            {
                auto const [idxSubmap, pointIdx] = itTrack->first;
                if (idxSubmap == idxKeyframe)
                    continue;
                A.row(idxClusterPt++) = keyframeSubmaps_[idxSubmap]->points_[pointIdx] - centroid;
            }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const Eigen::Vector3d planeNormal = svd.matrixV().col(2).normalized();

            // Check if plane is valid - all points need to be within threshold distance to plane
            bool isPlaneValid = true;
            double planeThickness = 0.0;
            for (size_t idxPt = 0; idxPt < numHistoricPoints; idxPt++)
            {
                const double pointToPlaneDist = std::abs(planeNormal.dot(A.row(idxPt)));
                if (pointToPlaneDist > config_.lidar_frontend.clustering.max_plane_thickness)
                {
                    isPlaneValid = false;
                    break;
                }
                else
                {
                    planeThickness += std::pow(pointToPlaneDist, 2);
                }
            }

            // Remove tracked cluster if plane is not valid
            if (!isPlaneValid)
            {
                itCluster = eraseClusterAndTracks(itCluster);
            }
            else
            {
                planeThickness /= n;
                clusterPlaneThickness_[clusterId] = planeThickness;
                clusterCenters_[clusterId] = std::make_shared<Eigen::Vector3d>(centroid);
                clusterNormals_[clusterId] = std::make_shared<Eigen::Vector3d>(planeNormal);
                ++itCluster;
            }
        }

        auto stopwatchClusteringEnd = std::chrono::high_resolution_clock::now();
        auto durationClustering = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatchClusteringEnd - stopwatchClusteringStart).count();
        std::cout << "::: [DEBUG] clustering and outlier rejection took " << durationClustering << " ms :::" << std::endl;

        // check if there are still tracks for the current keyframe after outlier rejection
        for (const auto &[idxTrack, _] : clusterTracks_)
        {
            if (idxTrack.first == idxKeyframe)
            {
                keyframeHasTracks = true;
                break;
            }
        }

        return keyframeHasTracks; // indicates if we lost tracking
    }

    Clusters::iterator MappingSystem::eraseClusterAndTracks(Clusters::iterator &itCluster)
    {
        // erase all tracks associated with this cluster
        for (ClusterTracks::iterator &itPoint : itCluster->second)
            clusterTracks_.erase(itPoint);
        // erase the cluster itself
        Clusters::iterator itClusterNxt = clusters_.erase(itCluster);
        // erase cluster properties if they exist (might not have been evaluated yet)
        if (clusterPlaneThickness_.find(itCluster->first) != clusterPlaneThickness_.end())
            clusterPlaneThickness_.erase(itCluster->first);
        if (clusterCenters_.find(itCluster->first) != clusterCenters_.end())
            clusterCenters_.erase(itCluster->first);
        if (clusterNormals_.find(itCluster->first) != clusterNormals_.end())
            clusterNormals_.erase(itCluster->first);
        return itClusterNxt;
    }

    void MappingSystem::removeTracksFromClusters(const u_int32_t &idxKeyframe)
    {
        // Remove all tracks from the marginalized keyframe
        for (auto itTrack = clusterTracks_.begin(); itTrack != clusterTracks_.end();)
        {
            if (itTrack->first.first == idxKeyframe) // if submap index matches
                itTrack = clusterTracks_.erase(itTrack);
            else
                ++itTrack;
        }

        // Remove clusters that no longer have enough points
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end();)
        {
            // NOTE: erase-remove -> (re-)move all previously deleted tracks to the end of the range (returns new logical end)
            // then erase all elements past the new logical end
            auto &trackIters = itCluster->second;
            trackIters.erase(
                std::remove_if(trackIters.begin(), trackIters.end(),
                               [idxKeyframe](const ClusterTracks::iterator &it)
                               {
                                   return it->first.first == idxKeyframe;
                               }),
                trackIters.end());

            // Remove cluster if too small
            if (trackIters.size() < config_.lidar_frontend.clustering.min_points)
                itCluster = eraseClusterAndTracks(itCluster);
            else
                ++itCluster;
        }
    }

    void MappingSystem::createAndUpdateFactors()
    {
        factorsToRemove_.clear();
        // Build residuals for active keyframe points from valid clusters
        for (auto itValidCluster = clusters_.begin(); itValidCluster != clusters_.end(); ++itValidCluster)
        {
            auto const &[clusterId, clusterTracks] = *itValidCluster;
            const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
            const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
            const double planeThickness = clusterPlaneThickness_[clusterId];

            // 1: Collect unique keyframe IDs involved in this cluster
            std::set<uint32_t> uniqueKeyframeIds;
            for (ClusterTracks::iterator itTrack : clusterTracks)
                // track is (idxSubmapIdxPoint, clusterId)
                uniqueKeyframeIds.insert(itTrack->first.first);

            // 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
            gtsam::KeyVector keys;
            keys.reserve(uniqueKeyframeIds.size());
            std::unordered_map<uint32_t, size_t> keyframeIdToKeyIdx;
            size_t keyIdx = 0;
            for (const uint32_t kfId : uniqueKeyframeIds)
            {
                keys.push_back(X(kfId));
                keyframeIdToKeyIdx[kfId] = keyIdx++;
            }

            // 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
            std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey;
            size_t totalPoints = 0;
            for (ClusterTracks::iterator itTrack : clusterTracks)
            {
                auto const [keyframeId, pointIdx] = itTrack->first;
                const size_t keyVecIdx = keyframeIdToKeyIdx[keyframeId];
                // scan points are passed to factor in world frame
                scanPointsPerKey[keyVecIdx].push_back(std::make_shared<Eigen::Vector3d>(
                    // transform point in keyframe from world frame to lidar frame
                    keyframePoses_[keyframeId]->transformTo(keyframeSubmaps_[keyframeId]->points_[pointIdx])));
                totalPoints++;
            }

            // Noise model for point-to-plane factor (one sigma per point measurement)
            gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(keys.size(), planeThickness);
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
        const uint32_t idxKeyframe = createKeyframeSubmap(w_X_curr_.pose().compose(imu_T_lidar_), tLastImu_, ptrNewSubmapVoxelized);
#ifndef DISABLEVIZ
        visualizer.addSubmap(idxKeyframe, w_X_curr_.pose().matrix(), ptrNewSubmapVoxelized);
        visualizer.waitForSpacebar();
#endif

        const bool isTracking = trackScanPointsToClusters(idxKeyframe);
        if (!isTracking)
        {
            std::cout << "::: [ERROR] lost tracking at keyframe " << idxKeyframe << " :::" << std::endl;
            systemState_ = SystemState::Recovery;
            return;
        }

        if (clusters_.size() == 0)
        {
            std::cout << "::: [ERROR] no valid clusters found in keyframe, tracking lost :::" << std::endl;
            systemState_ = SystemState::Recovery;
            return;
        }

        std::cout << "::: [DEBUG] outlier rejection completed (keeping " << clusters_.size()
                  << " clusters), proceeding to formulate tracking constraints :::" << std::endl;

        // NOTE: will internally update factorsToRemove to drop outdated smart factors
        // the outdated factors will be replaced by extended ones with additional tracks
        createAndUpdateFactors();

        // Add variables for the active keyframe
        newValues_.insert(X(idxKeyframe), w_X_curr_.pose());
        newValues_.insert(V(idxKeyframe), w_X_curr_.v());
        newValues_.insert(B(idxKeyframe), currBias_);

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
        smoother_.update(newSmootherFactors_, newValues_, newSmootherIndices_, factorsToRemove_);
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

        if (idxKeyframe > static_cast<uint32_t>(config_.backend.sliding_window_size))
        {
            uint32_t idxLowerBound = keyframeSubmaps_.begin()->first;
            uint32_t idxUpperbound = idxKeyframe - static_cast<uint32_t>(config_.backend.sliding_window_size);

            std::cout << "::: [DEBUG] marginalizing " << (idxUpperbound - idxLowerBound) << " keyframes :::" << std::endl;
            for (uint32_t idxMargiznalizedKeyframe = idxLowerBound; idxMargiznalizedKeyframe < idxUpperbound; ++idxMargiznalizedKeyframe)
            {
                if (keyframeSubmaps_.find(idxMargiznalizedKeyframe) != keyframeSubmaps_.end())
                {
                    removeTracksFromClusters(idxMargiznalizedKeyframe);
                    keyframeSubmaps_.erase(idxMargiznalizedKeyframe);
                    submapKDTrees_.erase(idxMargiznalizedKeyframe);
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
            const gtsam::Pose3 updatedPose = smoother_.calculateEstimate<gtsam::Pose3>(X(idxKf));
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
        const gtsam::Pose3 &keyframePose,
        double keyframeTimestamp,
        std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap)
    {
        // Increment keyframe counter
        const uint32_t idxNewKf = keyframeCounter_++;

        // Transform submap to its estimated pose in the world frame
        ptrKeyframeSubmap->Transform(keyframePose.matrix());
        keyframeSubmaps_[idxNewKf] = ptrKeyframeSubmap;
        keyframePoses_[idxNewKf] = std::make_shared<gtsam::Pose3>(keyframePose);
        keyframeTimestamps_[idxNewKf] = keyframeTimestamp;

        std::cout << "::: [DEBUG] created keyframe " << idxNewKf << " ("
                  << ptrKeyframeSubmap->points_.size() << " pts) at timestamp "
                  << keyframeTimestamp << " :::" << std::endl;

        // Initialize a KD-Tree for point cluster search
        submapKDTrees_[idxNewKf] = std::make_shared<open3d::geometry::KDTreeFlann>(*ptrKeyframeSubmap);
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

    void MappingSystem::updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &newWorldPose)
    {
        const gtsam::Pose3 deltaPose = keyframePoses_[keyframeIdx]->between(newWorldPose);
        keyframeSubmaps_[keyframeIdx]->Transform(deltaPose.matrix());
        keyframePoses_[keyframeIdx] = std::make_shared<gtsam::Pose3>(newWorldPose);
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
        std::cout << "::: [SUMMARY] Current clusters :::" << std::endl;
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end(); ++itCluster)
        {
            auto const &[clusterId, clusterPoints] = *itCluster;
            std::cout << "\tCluster " << clusterId << ": " << clusterPoints.size() << " points" << std::endl;
            // << "thickness " << clusterPlaneThickness_.at(clusterId) << std::endl;
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
        std::map<ClusterId, PointCluster> pointClusters;
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end(); ++itCluster)
        {
            auto const &[clusterId, clusterTracks] = *itCluster;
            PointCluster pointCluster;
            pointCluster.center = clusterCenters_.at(clusterId);
            pointCluster.normal = clusterNormals_.at(clusterId);
            pointClusters[clusterId] = pointCluster;
        }
        return pointClusters;
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
