#include <mapping/frontend/ImuFrontend.hpp>
#include <mapping/helpers.hpp> // Scan2PCD

namespace mapping
{
    gtsam::NavState ImuFrontend::preintegrateIMU(const States &states, Buffers &buffers)
    {
        // Lock the buffers for accessing values
        std::unique_lock<std::mutex> lockImuBuffer(buffers.getMtxImuBuffer());
        std::map<double, std::shared_ptr<ImuData>> &imuBuffer = buffers.getImuBuffer();

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

    void ImuFrontend::undistortScans(const States &states, Buffers &buffers, const MappingConfig &config)
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
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, config.point_filter.min_distance, config.point_filter.max_distance);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(config.lidar_frontend.voxel_size);

            // Buffer undistorted scan
            buffers.bufferScan(kf_T_scan, ptrPcdScanVoxelized);
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