/// @file
/// @ingroup frontend_imu
#include <mapping/frontend/ImuFrontend.hpp>
#include <mapping/helpers.hpp> // Scan2PCD

namespace mapping
{
    ImuFrontend::ImuFrontend()
    {
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

    void ImuFrontend::initializeFromStatic(
        const double &accVariance,
        const double &gyroVariance,
        const Eigen::Vector3d &gyroBiasMean)
    {
        if (isInitialized_)
            throw std::runtime_error("::: [ERROR] Tried to initialize (static) IMU frontend twice. :::");
        preintegrator_.params()->accelerometerCovariance = gtsam::I_3x3 * accVariance;
        preintegrator_.params()->gyroscopeCovariance = gtsam::I_3x3 * gyroVariance;
        gtsam::imuBias::ConstantBias priorImuBias{preintegrator_.biasHat().accelerometer(), gyroBiasMean};
        preintegrator_.resetIntegrationAndSetBias(priorImuBias);
        isInitialized_ = true;
    }

    gtsam::NavState ImuFrontend::preintegrateIMU(const States &states, Buffers &buffers)
    {
        if (!isInitialized_)
            throw std::runtime_error("::: [ERROR] Tried to preintegrate IMU measurements without initializing the IMU frontend. :::");
        // Lock the buffers for accessing values
        std::unique_lock<std::mutex> lockImuBuffer(buffers.getMtxImuBuffer());
        std::map<double, std::shared_ptr<ImuData>> &imuBuffer = buffers.getImuBuffer();

        // guard against empty buffer (caused by multi-threaded executor while CPU under load)
        if (imuBuffer.empty())
        {
            lockImuBuffer.unlock();
            return preintegrator_.predict(states.getPreintegrationRefState(), states.getCurrentBias());
        }

        // Perform preintegration to decide whether a new keyframe is needed
        for (auto imuIt = imuBuffer.begin(); imuIt != imuBuffer.end(); ++imuIt)
        {
            const auto [timestamp, u] = *imuIt;
            const double dt = imuIt == imuBuffer.begin() ? timestamp - states.tLastImu_ : timestamp - std::prev(imuIt)->first;
            // Integrate measurement
            preintegrator_.integrateMeasurement(u->acceleration, u->angular_velocity, dt);
        }

        // Save last imu timestamp and clear the buffer
        states.setLastImuTime(imuBuffer.rbegin()->first);
        imuBuffer.clear();
        lockImuBuffer.unlock();

        const gtsam::NavState w_X_propagated = preintegrator_.predict(
            states.getPreintegrationRefState(),
            states.getCurrentBias());
        return w_X_propagated;
    }

    void ImuFrontend::resetPreintegrator(const States &states)
    {
        states.setPreintegrationRefState(states.getCurrentState());
        preintegrator_.resetIntegrationAndSetBias(states.getCurrentBias());
    }

    void ImuFrontend::undistortScans(States &states, Buffers &buffers, const MappingConfig &config)
    {
        // Iterator to the newest lidar scan that can still be processed (older than latest imu timestamp)
        std::unique_lock<std::mutex> lockLidarBuffer(buffers.getMtxLidarBuffer());

        // Undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation
        // Delta pose to last submap at preintegrated state / last IMU time
        gtsam::Pose3 kf_T_prop = states.lastKeyframePose().between(states.getCurrentState().pose().compose(states.getImuToLidarExtrinsic()));

        const double tLastKeyframe = states.lastKeyframeTimestamp();
        // time delta between last keyframe and new preintegration
        const double dtPropToKeyframe = states.tLastImu_ - tLastKeyframe;

        // Pose delta of this scan to last keyframe
        // Compute pose delta to last submap at current scan time (extrapolate with constant velocity)
        // Linear and angular velocity are obtained from pose delta over preintegration time
        const gtsam::Vector3 angVel = gtsam::Rot3::Logmap(kf_T_prop.rotation()) / dtPropToKeyframe;
        const gtsam::Vector3 linVel = kf_T_prop.translation() / dtPropToKeyframe;

        std::map<double, std::shared_ptr<LidarData>> &lidarBuffer = buffers.getLidarBuffer();
        states.setLastScanTime(lidarBuffer.rbegin()->first);

        double tLastScan = tLastKeyframe;
        // Undistort between individual scans
        gtsam::Pose3 deltaPoseLastScanToKeyframe = gtsam::Pose3::Identity();
        for (auto it = lidarBuffer.begin(); it != lidarBuffer.end(); ++it)
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
            // undistort scan, if enabled
            if(config.lidar_frontend.undistort)
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
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, config.point_filter.min_distance, config.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(config.lidar_frontend.voxel_size);

            // Buffer undistorted scan, carrying the camera association forward
            buffers.bufferScan(kf_T_scan, ptrPcdScanVoxelized, scan->syncedCameraData);
            // update pose and time reference for computing delta between scans in next iteration
            deltaPoseLastScanToKeyframe = kf_T_scan;
            tLastScan = tScan;
        }

        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // Erase processed raw scan data and release buffer lock
        lidarBuffer.clear();
        lockLidarBuffer.unlock();
    }
}
