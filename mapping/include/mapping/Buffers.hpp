#pragma once

#ifndef MAPPING_BUFFERS_HPP_
#define MAPPING_BUFFERS_HPP_

#include <mapping/types.hpp>
#include <open3d/geometry/PointCloud.h>
#include <memory>
#include <mutex>
#include <list>
#include <map>

namespace mapping
{
    class Buffers
    {
    public:
        Buffers() = default;
        ~Buffers() = default;

        /// @brief Feed an IMU measurement to the system
        /// @param imu_data IMU acceleration and angular velocity
        /// @param timestamp Measurement timestamp
        void feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp);

        /// @brief Feed a LiDAR scan to the system
        /// @param lidar_data LiDAR points with per-point timestamps
        /// @param timestamp Scan timestamp
        void feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp);

        /// @brief Add a new (undistorted) scan pointcloud to the scan buffer (but don't apply pose transformation)
        /// @details The scan pointcloud is expected to be undistorted already, but all points lie in the scan frame.
        /// Use the scan buffer entries pose data to transform the scan to the desired frame.
        /// @param scanPoseToLastKeyframe Pose of the scan relative to last keyframe
        /// @param pcdScan Voxelized pointcloud of the scan
        void bufferScan(
            const gtsam::Pose3 &scanPoseToLastKeyframe,
            std::shared_ptr<open3d::geometry::PointCloud> pcdScan);

        /// @brief Enable or disable collection of marginalized submaps.
        /// Disabled by default to avoid unbounded memory growth in headless mode.
        void setCollectMarginalizedSubmaps(bool enable)
        {
            collectMarginalizedSubmaps_ = enable;
        };

        /// @brief Drain and return submaps that were marginalized since the last call.
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> getMarginalizedSubmaps();

        /// @brief Get mutexes for lock guard or unique lock.
        /// You should always lock the corresponding mutex before accessing the buffers.
        std::mutex &getMtxImuBuffer() { return mtxImuBuffer_; };
        std::mutex &getMtxLidarBuffer() { return mtxLidarBuffer_; };
        /// @brief Getters for the actual buffer data.
        std::map<double, std::shared_ptr<ImuData>> &getImuBuffer() { return imuBuffer_; };
        std::map<double, std::shared_ptr<LidarData>> &getLidarBuffer() { return lidarBuffer_; };
        std::list<ScanBuffer> &getScanBuffer() { return scanBuffer_; };

    private:
        // mutexes to lock buffer access
        std::mutex mtxLidarBuffer_, mtxImuBuffer_;
        // Buffers for incoming data
        std::map<double, std::shared_ptr<ImuData>> imuBuffer_;
        std::map<double, std::shared_ptr<LidarData>> lidarBuffer_;
        /// @brief Undistorted scans that have not yet been merged into keyframe submaps.
        std::list<ScanBuffer> scanBuffer_;
        /// @brief whether to keep marginalized submap PCDs for visualization interfaces.
        bool collectMarginalizedSubmaps_ = false;
        /// @brief marginalized submap PCDs, cleared on each call to getMarginalizedSubmaps()
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> marginalizedSubmaps_;
    };
}

#endif // MAPPING_BUFFERS_HPP_