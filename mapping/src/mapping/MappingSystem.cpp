#include <mapping/MappingSystem.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <unordered_map>

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
        : smoother_(static_cast<double>(kSlidingWindowSize)),
          systemState_(SystemState::Initializing),
          keyframeCounter_(0),
          tLastImu_(0.0),
          imu_T_lidar_(gtsam::Rot3::Identity(), gtsam::Point3(-0.011, -0.02329, 0.04412)),
          lidarTimeOffset_(0.0),
          clusterIdCounter_(0)
    {
        // Initialize IMU preintegration parameters
        auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        // TODO: set sensible values for Livox Mid360 IMU here
        constexpr double accelerometerNoise = 0.15;
        constexpr double gyroscopeNoise = 0.314;
        params->accelerometerCovariance = gtsam::I_3x3 * std::pow(accelerometerNoise, 2);
        params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(gyroscopeNoise, 2);
        params->integrationCovariance = gtsam::I_3x3 * 1e-6;
        gtsam::imuBias::ConstantBias priorImuBias{};
        preintegrator_ = gtsam::PreintegratedCombinedMeasurements(params, priorImuBias);
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
        std::cout << "SLAM has " << imuBuffer_.size() << " IMU messages and "
                  << lidarBuffer_.size() << " LIDAR messages buffered." << std::endl;

        switch (systemState_)
        {
        case SystemState::Initializing:
        {
            const double maxBufferTime = lidarBuffer_.rbegin()->first;
            if (maxBufferTime >= kInitTimeWindow)
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
        for (auto it = lidarBuffer_.begin(); it != lidarBufferEndIt; ++it)
        {
            open3d::geometry::PointCloud pcdScan = Scan2PCD(it->second, kMinPointDist, kMaxPointDist);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScan = pcdScan.VoxelDownSample(kVoxelSize);
            ptrPcdScan->Transform(w_T_i0.matrix() * imu_T_lidar_.matrix());
            newSubmap += *ptrPcdScan;
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(kVoxelSize);
        const uint32_t idxNewKF = createKeyframeSubmap(w_T_i0.compose(imu_T_lidar_), tLastImu_, ptrNewSubmapVoxelized);

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

        std::cout << "::: [DEBUG] tracking with " << imuBuffer_.size() << " IMU measurements and "
                  << lidarBuffer_.size() << " LiDAR scans :::" << std::endl;

        // Perform preintegration to decide whether a new keyframe is needed
        for (auto imuIt = imuBuffer_.begin(); imuIt != imuBuffer_.end(); ++imuIt)
        {
            const auto [timestamp, u] = *imuIt;
            const double dt = imuIt == imuBuffer_.begin() ? timestamp - tLastImu_ : timestamp - std::prev(imuIt)->first;
            // Integrate measurement
            preintegrator_.integrateMeasurement(u->acceleration, u->angular_velocity, dt);
        }
        std::cout << "::: [DEBUG] IMU preintegration completed :::" << std::endl;

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
        std::cout << "::: [DEBUG] tLastIMU " << tLastImu_ << ", tLastLiDAR "
                  << (lidarBuffer_.size() ? lidarBuffer_.rbegin()->first : 0) << " :::" << std::endl;

        // Undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation
        // Delta pose to last submap at preintegrated state / last IMU time
        gtsam::Pose3 kf_T_prop = lastKeyframePose().inverse().compose(w_X_curr_.pose().compose(imu_T_lidar_));
        std::cout << "::: [DEBUG] has " << keyframeTimestamps_.size() << " keyframes :::" << std::endl;

        const double tLastKeyframe = keyframeTimestamps_.rbegin()->second;
        const double dtPropToKeyframe = tLastImu_ - tLastKeyframe;
        std::cout << "::: [DEBUG] begin undistorting " << lidarBuffer_.size() << " LiDAR scans :::" << std::endl;

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
            std::cout << "::: [DEBUG] undistorted LiDAR scan with " << scan->points.size() << " points :::" << std::endl;

            // Convert scan to pointcloud, voxelize, transform to keyframe pose and add to submap
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, kMinPointDist, kMaxPointDist);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(kVoxelSize);
            gtsam::Pose3 scanPoseInWorld = keyframePoses_.rbegin()->second->compose(kf_T_scan);
            ptrPcdScanVoxelized->Transform(scanPoseInWorld.matrix());

            // Buffer undistorted scan
            bufferScan(kf_T_scan, ptrPcdScanVoxelized);
            deltaPoseLastScanToKeyframe = kf_T_scan;
        }
        std::cout << "::: [DEBUG] completed undistorting LiDAR scans, proceeding to identify keyframe :::" << std::endl;

        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // Erase processed raw scan data and release buffer lock
        lidarBuffer_.clear();
        lockLidarBuffer.unlock();
    }

    void MappingSystem::trackScanPointsToClusters(const uint32_t &idxKeyframe)
    {
        std::cout
            << "::: [DEBUG] identifying same-plane point clusters among keyframe submaps :::" << std::endl;

        // Search for same-plane point clusters among all keyframes
        const std::shared_ptr<open3d::geometry::PointCloud> pcdQuery = keyframeSubmaps_[idxKeyframe];
        std::vector<int> knnIndices(kKnnMaxNeighbors);
        std::vector<double> knnDists(kKnnMaxNeighbors);

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
                    pcdQuery->points_[idxPt], kKnnRadius, kKnnMaxNeighbors, knnIndices, knnDists);

                if (knnFound > 4)
                {
                    ClusterId clusterId = INVALID_CLUSTER_ID;
                    ClusterId tempNewCluster = clusterIdCounter_ + 1;
                    // Clusters that need to be merged (polled & processed after search)
                    std::vector<ClusterId> duplicateClusters;

                    // Identify or assign cluster to every NN returned point
                    for (int i = 0; i < knnFound; ++i)
                    {
                        SubmapIdxPointIdx idxOther{idxOtherSubmap, knnIndices[i]};
                        if (clusterTracks_.find(idxOther) != clusterTracks_.end())
                        {
                            clusterTracks_[idxOther] = tempNewCluster;
                        }
                        else
                        {
                            // Point belongs to a single existing cluster
                            if (clusterId == INVALID_CLUSTER_ID)
                            {
                                clusterId = clusterTracks_[idxOther];
                                // May have assigned temp cluster, which needs to be replaced
                                duplicateClusters.push_back(tempNewCluster);
                            }
                            else // Point belongs to multiple existing clusters, need to merge
                            {
                                duplicateClusters.push_back(clusterTracks_[idxOther]);
                            }
                        }

                        // If a new cluster was identified, increment the counter
                        if (clusterId == INVALID_CLUSTER_ID)
                            clusterIdCounter_ = clusterId = tempNewCluster;
                        // This point belongs to an existing cluster
                        else // Merge all duplicate clusters with the identified one
                        {
                            for (const ClusterId &duplicateId : duplicateClusters)
                                for (auto &entry : clusterTracks_)
                                    if (entry.second == duplicateId)
                                        entry.second = clusterId;
                        }

                        // Add the current point to the identified cluster
                        SubmapIdxPointIdx idxCurrent{idxKeyframe, idxPt};
                        clusterTracks_[idxCurrent] = clusterId;
                    }
                }
            }
        }

        for (auto itTrack = clusterTracks_.begin(); itTrack != clusterTracks_.end(); ++itTrack)
        {
            auto const &[idxPoint, clusterId] = *itTrack;
            if (clusters_.find(clusterId) == clusters_.end())
                clusters_[clusterId] = std::vector<ClusterTracks::iterator>{itTrack};
            else
                clusters_[clusterId].push_back(itTrack);
        }

        // Remove clusters that are too small
        for (auto itCluster = clusters_.begin(); itCluster != clusters_.end();)
        {
            auto const &[clusterId, clusterPoints] = *itCluster;
            if (clusterPoints.size() < kMinClusterSize)
                itCluster = eraseClusterAndTracks(itCluster);
            else
                ++itCluster;
        }
        std::cout << "::: [DEBUG] identified " << clusters_.size() << " clusters, proceeding with outlier rejection :::" << std::endl;

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
                if (pointToPlaneDist > kThreshPlaneValid)
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
    }

    Clusters::iterator MappingSystem::eraseClusterAndTracks(Clusters::iterator &itCluster)
    {
        // erase all tracks associated with this cluster
        for (ClusterTracks::iterator &itPoint : itCluster->second)
            clusterTracks_.erase(itPoint);
        // erase the cluster itself
        Clusters::iterator itClusterNxt = clusters_.erase(itCluster);
        return itClusterNxt;
    }

    void MappingSystem::track()
    {
        const gtsam::NavState w_X_propagated = preintegrateIMU();
        const double positionDiff = (w_X_propagated.pose().translation() - lastKeyframePose().translation()).norm();
        w_X_curr_ = w_X_propagated;
        // undistort all scans and move them to the scanBuffer
        undistortScans();
        if (positionDiff < kThreshNewKeyframeDist)
            return;
        std::cout << "::: [INFO] identified keyframe, creating new submap :::" << std::endl;
        // Create new keyframe
        // Merge buffered scans together & create new keyframe submap
        open3d::geometry::PointCloud newSubmap;
        for (const ScanBuffer &scan : scanBuffer_)
        {
            // Move all scans to same origin
            scan.pcd->Transform(scan.kf_T_scan->matrix());
            newSubmap += *(scan.pcd);
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(kVoxelSize);
        const uint32_t idxKeyframe = createKeyframeSubmap(w_X_curr_.pose().compose(imu_T_lidar_), tLastImu_, ptrNewSubmapVoxelized);
#ifndef DISABLEVIZ
        visualizer.addSubmap(idxKeyframe, w_X_curr_.pose().matrix(), ptrNewSubmapVoxelized);
        visualizer.waitForSpacebar();
#endif

        trackScanPointsToClusters(idxKeyframe);

        if (clusters_.size() == 0)
        {
            std::cout << "::: [ERROR] no valid clusters found in keyframe, tracking lost :::" << std::endl;
            systemState_ = SystemState::Recovery;
            return;
        }

        std::cout << "::: [DEBUG] outlier rejection completed (keeping " << clusters_.size()
                  << " clusters), proceeding to formulate tracking constraints :::" << std::endl;

        // Build residuals for active keyframe points from valid clusters
        for (auto itValidCluster = clusters_.begin(); itValidCluster != clusters_.end(); ++itValidCluster)
        {
            auto const &[clusterId, clusterTracks] = *itValidCluster;
            const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
            const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
            const double planeThickness = clusterPlaneThickness_[clusterId];

            // Step 1: Collect unique keyframe IDs involved in this cluster
            std::set<uint32_t> uniqueKeyframeIds;
            for (ClusterTracks::iterator itTrack : clusterTracks)
            {
                // track is (idxSubmapIdxPoint, clusterId)
                uniqueKeyframeIds.insert(itTrack->first.first);
            }

            // Step 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
            gtsam::KeyVector keys;
            keys.reserve(uniqueKeyframeIds.size());
            std::unordered_map<uint32_t, size_t> keyframeIdToKeyIdx;
            size_t keyIdx = 0;
            for (const uint32_t kfId : uniqueKeyframeIds)
            {
                keys.push_back(X(kfId));
                keyframeIdToKeyIdx[kfId] = keyIdx++;
            }

            // Step 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
            std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey;
            size_t totalPoints = 0;
            for (ClusterTracks::iterator itTrack : clusterTracks)
            {
                auto const [keyframeId, pointIdx] = itTrack->first;
                const size_t keyVecIdx = keyframeIdToKeyIdx[keyframeId];
                // Point in LiDAR frame = world point - keyframe translation
                scanPointsPerKey[keyVecIdx].push_back(std::make_shared<Eigen::Vector3d>(
                    keyframeSubmaps_[keyframeId]->points_[pointIdx] - keyframePoses_[keyframeId]->translation()));
                totalPoints++;
            }

            // Noise model for point-to-plane factor (one sigma per point measurement)
            gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(totalPoints, std::sqrt(planeThickness));

            // Create a point-to-plane factor for this cluster
            newSmootherFactors_.add(boost::make_shared<PointToPlaneFactor>(
                keys,
                imu_T_lidar_,
                scanPointsPerKey,
                clusterNormal,
                clusterNormal->dot(*clusterCenter),
                noiseModel));
        }

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

        smoother_.update(newSmootherFactors_, newValues_, newSmootherIndices_);
        smoother_.print("Smoother after update:\n");
        resetNewFactors();

        // Extract estimated state
        w_X_curr_ = gtsam::NavState{
            smoother_.calculateEstimate<gtsam::Pose3>(X(idxKeyframe)),
            smoother_.calculateEstimate<gtsam::Vector3>(V(idxKeyframe))};
        currBias_ = smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(B(idxKeyframe));
        w_X_curr_.print();

        // Reset preintegrator
        preintegrator_.resetIntegrationAndSetBias(currBias_);

        if (idxKeyframe > kSlidingWindowSize)
        {
            uint32_t idxLowerBound = keyframeSubmaps_.begin()->first;
            uint32_t idxUpperbound = idxKeyframe - kSlidingWindowSize;

            std::cout << "::: [DEBUG] marginalizing " << (idxUpperbound - idxLowerBound) << " keyframes :::" << std::endl;
            for (uint32_t idx = idxLowerBound; idx < idxUpperbound; ++idx)
            {
                if (keyframeSubmaps_.find(idx) != keyframeSubmaps_.end())
                {
                    keyframeSubmaps_.erase(idx);
                    submapKDTrees_.erase(idx);
                    keyframePoses_.erase(idx);
                    keyframeTimestamps_.erase(idx);
#ifndef DISABLEVIZ
                    visualizer.removeSubmap(idx);
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
            visualizer.waitForSpacebar();
#endif
        }
    }

    void MappingSystem::resetNewFactors()
    {
        newSmootherFactors_.resize(0);
        newValues_.clear();
        newSmootherIndices_.clear();
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
    }

} // namespace mapping
