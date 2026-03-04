#include <mapping/Buffers.hpp>

namespace mapping
{
    void Buffers::feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp)
    {
        std::lock_guard<std::mutex> lock(mtxImuBuffer_);
        imuBuffer_[timestamp] = imu_data;
    }

    void Buffers::feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp)
    {
        std::lock_guard<std::mutex> lock(mtxLidarBuffer_);
        lidarBuffer_[timestamp] = lidar_data;
    }

    void Buffers::bufferScan(
        const gtsam::Pose3 &scanPoseToLastKeyframe,
        std::shared_ptr<open3d::geometry::PointCloud> pcdScan)
    {
        scanBuffer_.push_back(ScanBuffer{pcdScan, std::make_shared<gtsam::Pose3>(scanPoseToLastKeyframe)});
    }
}