#include <mapping/MappingSystem.hpp>
#include <mapping/helpers.hpp>

#include <csignal>
#include <iostream>

namespace mapping
{

    MappingSystem::MappingSystem()
        : MappingSystem(MappingConfig())
    {
    }

    MappingSystem::MappingSystem(const MappingConfig &config)
        : config_(config),
          imu_T_lidar_(config.extrinsics.imu_T_lidar.toPose3())
    {
        states_.setImuToLidarExtrinsic(imu_T_lidar_);
        smoother_.reset(config_);
    }

    void MappingSystem::setConfig(const MappingConfig &config)
    {
        config_ = config;
        imu_T_lidar_ = config_.extrinsics.imu_T_lidar.toPose3();
        states_.setImuToLidarExtrinsic(imu_T_lidar_);
        smoother_.reset(config_);
    }

    void MappingSystem::feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp)
    {
        buffers_.feedImu(imu_data, timestamp);
    }

    void MappingSystem::feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp)
    {
        buffers_.feedLidar(lidar_data, timestamp);
    }

    void MappingSystem::update()
    {
        switch (states_.getLifecycleState())
        {
        case SystemState::Initializing:
        {
            const double maxBufferTime = buffers_.getLidarBuffer().rbegin()->first;
            if (maxBufferTime >= config_.backend.init_time_window)
            {
                initializeSystem();
                std::cout << "Initialization complete. Switching to tracking state." << std::endl;
                states_.setLifecycleState(SystemState::Tracking);
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
            // std::raise(SIGINT);
            return;
        }
        }
    }

    void MappingSystem::initializeSystem()
    {
        std::lock_guard<std::mutex> lockImuBuffer(buffers_.getMtxImuBuffer());
        std::lock_guard<std::mutex> lockLidarBuffer(buffers_.getMtxLidarBuffer());

        std::map<double, std::shared_ptr<ImuData>> &imuBuffer = buffers_.getImuBuffer();
        std::map<double, std::shared_ptr<LidarData>> &lidarBuffer = buffers_.getLidarBuffer();

        const double tInit = lidarBuffer.rbegin()->first;
        // upper bound key for all imu samples up to and including tInit
        auto imuBufferEndIt = imuBuffer.upper_bound(tInit);
        auto lidarBufferEndIt = lidarBuffer.upper_bound(tInit);

        // compute mean accelerometer reading for gravity alignment
        const double numImuSamples = static_cast<double>(std::distance(imuBuffer.begin(), imuBufferEndIt));
        Eigen::Vector3d accMean = Eigen::Vector3d::Zero();
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
            accMean += it->second->acceleration;
        accMean /= numImuSamples;
        std::cout << "Mean accelerometer measurement during initialization: " << std::endl
                  << accMean.transpose() << std::endl;

        /* Build gravity-aligned global reference frame (only roll & pitch are observable).
         * The z-axis is aligned with gravity, x and y are obtained by orthogonal projection.
         */
        Eigen::Vector3d zAxis = accMean.normalized();
        Eigen::Vector3d xAxis = ((Eigen::Matrix3d::Identity() - zAxis * zAxis.transpose()) * Eigen::Vector3d::UnitX()).normalized();
        Eigen::Vector3d yAxis = zAxis.cross(xAxis);

        Eigen::Matrix3d w_R_i0;
        w_R_i0.col(0) = xAxis;
        w_R_i0.col(1) = yAxis;
        w_R_i0.col(2) = zAxis;
        w_R_i0.transposeInPlace();

        gtsam::Pose3 w_T_i0{gtsam::Rot3(w_R_i0), gtsam::Point3(0, 0, 0)};
        gtsam::Pose3 w_T_l0 = w_T_i0.compose(imu_T_lidar_);

        // create initial submap from all buffered lidar scans
        open3d::geometry::PointCloud newSubmap;
        for (auto it = lidarBuffer.begin(); it != lidarBufferEndIt; ++it)
        {
            open3d::geometry::PointCloud pcdScan = Scan2PCD(it->second, config_.point_filter.min_distance, config_.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScan = pcdScan.VoxelDownSample(config_.lidar_frontend.voxel_size);
            ptrPcdScan->Transform(w_T_l0.matrix());
            newSubmap += *ptrPcdScan;
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(config_.lidar_frontend.voxel_size);

        // modifies states_: stores keyframe submap, pose, timestamp, increments keyframe counter
        const uint32_t idxNewKF = states_.createKeyframeSubmap(w_T_l0, 0.0, ptrNewSubmapVoxelized);
        // modifies featureManager_: creates premature clusters for each point in the keyframe
        featureManager_.createNewClusters(states_, idxNewKF, /*voxelSize=*/0);
        lidarBuffer.clear();

        // compute gyroscope bias estimate and sensor noise variances from static period
        Eigen::Vector3d gyroBiasMean = Eigen::Vector3d::Zero();
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
            gyroBiasMean += it->second->angular_velocity;
        gyroBiasMean /= numImuSamples;

        double accVariance = 0.0, gyroVariance = 0.0;
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
        {
            accVariance += (it->second->acceleration - accMean).squaredNorm();
            gyroVariance += (it->second->angular_velocity - gyroBiasMean).squaredNorm();
        }
        accVariance /= (numImuSamples - 1.0);
        gyroVariance /= (numImuSamples - 1.0);

        // modifies imuFrontend_: calibrates preintegrator noise and resets with computed bias
        imuFrontend_.initializeFromStatic(accVariance, gyroVariance, gyroBiasMean);

        // build prior factors for the first keyframe
        gtsam::imuBias::ConstantBias priorImuBias{Eigen::Vector3d::Zero(), gyroBiasMean};
        gtsam::NonlinearFactorGraph priors;
        gtsam::Vector6 priorPoseSigma;
        priorPoseSigma << 0.0175, 0.0175, 0.0873, 0.1, 0.1, 0.1; // rpy xyz
        priors.addPrior(X(idxNewKF), w_T_i0, gtsam::noiseModel::Diagonal::Sigmas(priorPoseSigma));
        priors.addPrior(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Isotropic::Sigma(3, 1e-2));
        const gtsam::noiseModel::Diagonal::shared_ptr biasPriorNoise =
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
        priors.addPrior(B(idxNewKF), priorImuBias, biasPriorNoise);

        // modifies smoother_: buffers priors, initial values and timestamps for first update
        gtsam::NavState x0{w_T_i0, gtsam::Vector3::Zero()};
        smoother_.setPriors(idxNewKF, priors, x0, priorImuBias);

        // set initial navigation state
        states_.setCurrentState(x0);
        states_.setPreintegrationRefState(x0);

        // store last IMU timestamp and clear buffer
        states_.setLastImuTime(imuBufferEndIt->first);
        imuBuffer.clear();
    }

    void MappingSystem::track()
    {
        /* Preintegrate all buffered IMU measurements and propagate the navigation state.
         * Modifies imuFrontend_ (preintegrator), buffers_ (clears IMU buffer),
         * and states_ (updates tLastImu_).
         */
        gtsam::NavState w_X_propagated = imuFrontend_.preintegrateIMU(states_, buffers_);

        // check if motion since last keyframe exceeds thresholds
        const gtsam::Pose3 &lastKfPose = states_.lastKeyframePose();
        const double positionDiff = (w_X_propagated.pose().translation() - lastKfPose.translation()).norm();
        const double angleDiff = (lastKfPose.rotation().between(w_X_propagated.pose().rotation())).axisAngle().second;
        states_.setCurrentState(w_X_propagated);

        /* Undistort all scans using constant-velocity motion model and buffer them.
         * Modifies buffers_ (fills scan buffer, clears lidar buffer)
         * and states_ (updates tLastScan_).
         */
        imuFrontend_.undistortScans(states_, buffers_, config_);

        const std::size_t scansSinceLastKeyframe = buffers_.numScansSinceLastKeyframe();
        if (
            positionDiff < config_.lidar_frontend.keyframe.thresh_distance && angleDiff < config_.lidar_frontend.keyframe.thresh_angle && scansSinceLastKeyframe < config_.lidar_frontend.keyframe.thresh_elapsed_scans)
            return;

        std::cout << "::: [INFO] identified keyframe, creating new submap :::" << std::endl;
        std::cout << "::: [DEBUG] position diff: " << positionDiff
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_distance << "), angle diff: " << angleDiff
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_angle << "), scans elapsed: " << scansSinceLastKeyframe
                  << " (thresh " << config_.lidar_frontend.keyframe.thresh_elapsed_scans << ") :::" << std::endl;

        // modifies buffers_: clears scan buffer after merging all buffered scans
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized =
            lidarFrontend_.accumulateUndistortedScans(states_, buffers_, config_);

        const gtsam::Pose3 world_T_lidar = states_.getCurrentState().pose().compose(imu_T_lidar_);
        // modifies states_: stores new keyframe submap, pose, timestamp, increments counter
        const uint32_t idxKeyframe = states_.createKeyframeSubmap(world_T_lidar, states_.tLastScan_, ptrNewSubmapVoxelized);

        /* Marginalize keyframes outside the sliding window BEFORE tracking,
         * so that the smoother update won't reference dissociated variables.
         * Modifies featureManager_ (creates marginalization factors, removes associations)
         * and states_ (removes marginalized keyframes, optionally archives submaps).
         */
        marginalizeKeyframesOutsideSlidingWindow(idxKeyframe);

        // modifies featureManager_: updates cluster states, parameters and point associations via KNN tracking
        const bool isTracking = lidarFrontend_.trackScanPointsToClusters(idxKeyframe, states_, featureManager_, config_);

        // guard pruning to avoid premature removal before sliding window fills up
        if (idxKeyframe >= static_cast<uint32_t>(config_.backend.sliding_window_size))
            // modifies featureManager_: erases clusters marked as Pruned
            featureManager_.pruneClusters(idxKeyframe);

        if (!isTracking)
        {
            std::cout << "::: [ERROR] lost tracking at keyframe " << idxKeyframe << " :::" << std::endl;
            states_.setLifecycleState(SystemState::Recovery);
            return;
        }

        /* Build factors and optimize. The feature manager accumulates marginalization factors
         * from the prior marginalization step and combines them with newly created/updated
         * tracking factors. std::exchange is used internally to clear the factor buffers.
         */
        // modifies featureManager_: creates/updates/removes LiDAR factors, clears internal factor buffers
        std::pair<gtsam::NonlinearFactorGraph, gtsam::FactorIndices> featureFactorsAndRemovals =
            featureManager_.createAndUpdateFactors(states_, smoother_.getFactors());
        gtsam::NonlinearFactorGraph &featureFactors = featureFactorsAndRemovals.first;
        gtsam::FactorIndices &factorsToRemove = featureFactorsAndRemovals.second;

        gtsam::CombinedImuFactor imuFactor = imuFrontend_.createPreintegrationFactor(idxKeyframe - 1, idxKeyframe);

        /* Run iSAM2 update, extract state estimate, and update all keyframe submap poses.
         * Modifies states_: smootherEstimate, currentState, currentBias,
         * preintegrationRefState, and all keyframe submap pointclouds.
         */
        smoother_.updateAndOptimizeGraph(idxKeyframe, states_, config_, imuFactor, featureFactors, factorsToRemove);

        featureManager_.summarizeFactors(smoother_.getFactors());

        std::cout << "Current bias: ";
        states_.getCurrentBias().print();

        // modifies imuFrontend_: resets preintegrator with updated bias from smoother estimate
        imuFrontend_.resetPreintegrator(states_);

        // insert new clusters from a lagged keyframe to allow maturation before tracking
        if (idxKeyframe > config_.lidar_frontend.clustering.insert_lag + 1)
            // modifies featureManager_: creates new premature clusters from the lagged keyframe
            featureManager_.createNewClusters(
                states_,
                idxKeyframe - config_.lidar_frontend.clustering.insert_lag,
                config_.lidar_frontend.clustering.sampling_voxel_size);
    }

    void MappingSystem::marginalizeKeyframesOutsideSlidingWindow(const uint32_t &idxKeyframe)
    {
        if (idxKeyframe <= static_cast<uint32_t>(config_.backend.sliding_window_size))
            return;

        const uint32_t idxUpperbound = idxKeyframe - static_cast<uint32_t>(config_.backend.sliding_window_size);
        uint32_t idxMarginalize = states_.getKeyframeSubmaps().begin()->first;
        while (idxMarginalize < idxUpperbound)
        {
            if (states_.getKeyframeSubmaps().find(idxMarginalize) != states_.getKeyframeSubmaps().end())
            {
                gtsam::Values markovBlanket = states_.getMarkovBlanketForKeyframe(idxMarginalize, idxKeyframe);
                std::cout << "::: [INFO] marginalizing keyframe " << idxMarginalize << ", outside of sliding window :::" << std::endl;
                // modifies featureManager_: creates marginalization factors, removes point associations, updates cluster states
                featureManager_.removeKeyframeFromClusters(idxMarginalize, markovBlanket);
                // modifies states_: removes keyframe data, optionally archives submap
                states_.removeKeyframe(idxMarginalize);
            }
            idxMarginalize++;
        }
    }

    // --- public query methods ---

    SlidingWindowStates MappingSystem::getStates() const
    {
        SlidingWindowStates states;
        const gtsam::Values &smootherEstimate = states_.getSmootherEstimate();
        if (smootherEstimate.empty())
            return states;
        std::cout << ":::" << std::endl;
        for (const auto &[idxKf, _] : states_.getKeyframeSubmaps())
        {
            try
            {
                const gtsam::Pose3 kfPose = smootherEstimate.at(X(idxKf)).cast<gtsam::Pose3>();
                const gtsam::Vector3 kfVel = smootherEstimate.at(V(idxKf)).cast<gtsam::Vector3>();
                states[idxKf] = NavStateStamped{gtsam::NavState{kfPose, kfVel}, states_.getKeyframeTimestamps().at(idxKf)};
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
        const std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &submaps = states_.getKeyframeSubmaps();
        if (submaps.empty())
        {
            std::cerr << "::: [WARNING] no keyframe submaps available :::" << std::endl;
            return nullptr;
        }
        return submaps.rbegin()->second;
    }

    std::map<ClusterId, PointCluster> MappingSystem::getCurrentClusters() const
    {
        std::map<ClusterId, PointCluster> currentClusters;
        for (const auto &[clusterId, clusterPoints] : featureManager_.clusters_)
        {
            if (!featureManager_.isClusterValid(clusterId))
                continue;
            PointCluster clusterRepresentation{
                featureManager_.clusterCenters_.at(clusterId),
                featureManager_.clusterNormals_.at(clusterId)};
            currentClusters.emplace(clusterId, clusterRepresentation);
        }
        return currentClusters;
    }

    uint32_t MappingSystem::getKeyframeCount() const
    {
        return states_.getLatestKeyframeIdx();
    }

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> MappingSystem::getMarginalizedSubmaps()
    {
        return states_.getMarginalizedSubmaps();
    }

    void MappingSystem::setCollectMarginalizedSubmaps(bool enable)
    {
        states_.setCollectMarginalizedSubmaps(enable);
    }

} // namespace mapping
