#pragma once

#ifndef MAPPING_MAPPINGSYSTEM_HPP_
#define MAPPING_MAPPINGSYSTEM_HPP_

#include <Eigen/Dense>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <Visualization.hpp>

#include <list>
#include <map>
#include <memory>
#include <mutex>

namespace mapping
{

    /// @brief IMU measurement data
    struct ImuData
    {
        Eigen::Vector3d acceleration;
        Eigen::Vector3d angular_velocity;
    };

    /// @brief LiDAR scan data with per-point timestamps
    struct LidarData
    {
        std::vector<Eigen::Vector3d> points;
        std::vector<double> offset_times;
    };

    /// @brief Buffered scan data for keyframe creation
    struct ScanBuffer
    {
        /// @brief Pointcloud of undistorted (and voxelized) scan
        std::shared_ptr<open3d::geometry::PointCloud> pcd;
        /// @brief Pose of the scan w.r.t. the latest keyframe pose
        std::shared_ptr<gtsam::Pose3> kf_T_scan;
    };

    /// @brief System lifecycle states
    enum class SystemState
    {
        Initializing,
        Tracking,
        Recovery,
    };

    /// @brief Indexing of points in a submap for building point clusters
    /// @details Uses `int` for point index because that's what Open3D's KNN search returns.
    using SubmapIdxPointIdx = std::pair<uint32_t, int>;

    /// @brief Type used to identify point clusters
    using ClusterId = uint32_t;

    /// @brief Invalid cluster ID sentinel value
    constexpr ClusterId INVALID_CLUSTER_ID = std::numeric_limits<ClusterId>::max();

    /// @brief Convert LidarData to Open3D PointCloud (no undistortion).
    /// @param lidar_data The raw LiDAR scan, potentially undistorted.
    /// @param minPointDist Minimum point distance to include
    /// @param maxPointDist Maximum point distance to include
    /// @return an Open3D PointCloud, with scan timestamp info removed.
    open3d::geometry::PointCloud Scan2PCD(
        const std::shared_ptr<LidarData> &lidar_data,
        double minPointDist,
        double maxPointDist);

    /// @brief LiDAR-Inertial Odometry and Mapping System
    /// @details This class handles IMU preintegration, LiDAR scan processing,
    /// keyframe management, and factor graph optimization using a fixed-lag smoother.
    /// NOTE: This class should not use ROS-specific code.
    class MappingSystem
    {
    public:
        MappingSystem();
        ~MappingSystem() = default;

        /// @brief Feed an IMU measurement to the system
        /// @param imu_data IMU acceleration and angular velocity
        /// @param timestamp Measurement timestamp
        void feedImu(const std::shared_ptr<ImuData> &imu_data, double timestamp);

        /// @brief Feed a LiDAR scan to the system
        /// @param lidar_data LiDAR points with per-point timestamps
        /// @param timestamp Scan timestamp
        void feedLidar(const std::shared_ptr<LidarData> &lidar_data, double timestamp);

        /// @brief Process buffered measurements and update state estimate
        void update();

    private:
        /// @brief Static state (assumption-based) system initialization
        /// @details
        /// - Uses all IMU measurements up to the first LIDAR timestamp
        /// - Computes mean accelerometer and gyro measurements
        /// - Builds initial orientation from mean accelerometer (gravity alignment)
        /// - Creates initial keyframe submap from all buffered LIDAR scans
        void initializeSystem();

        /// @brief Main tracking loop - process IMU and LiDAR data
        void track();

        /// @brief Reset factor graph buffers for next iteration
        void resetNewFactors();

        /// @brief Store new keyframe submap and initialized pose
        /// @param keyframePose Pose of the keyframe in world frame
        /// @param keyframeTimestamp Timestamp of the keyframe
        /// @param ptrKeyframeSubmap Pointcloud of the keyframe submap
        /// @return Index of the newly created keyframe
        uint32_t createKeyframeSubmap(
            const gtsam::Pose3 &keyframePose,
            double keyframeTimestamp,
            std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap);

        /// @brief Get the pose of the most recent keyframe
        gtsam::Pose3 lastKeyframePose() const;

        /// @brief Update a keyframe's submap to a new world pose
        /// @param keyframeIdx Index of the keyframe to update
        /// @param newWorldPose New pose in world frame
        void updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &newWorldPose);

        /// @brief Add a new (undistorted) scan pointcloud to the scan buffer
        /// @param scanPoseToLastKeyframe Pose of the scan relative to last keyframe
        /// @param pcdScan Voxelized pointcloud of the scan
        void bufferScan(
            const gtsam::Pose3 &scanPoseToLastKeyframe,
            std::shared_ptr<open3d::geometry::PointCloud> pcdScan);

    private:
        // Input data buffers
        std::map<double, std::shared_ptr<ImuData>> imuBuffer_;
        std::map<double, std::shared_ptr<LidarData>> lidarBuffer_;

        // IMU preintegrator
        gtsam::PreintegratedCombinedMeasurements preintegrator_;

        // Fixed lag smoother
        gtsam::BatchFixedLagSmoother smoother_;

        // Factors, nodes and timestamps to add to the graph
        gtsam::NonlinearFactorGraph newSmootherFactors_;
        gtsam::Values newValues_;
        gtsam::FixedLagSmoother::KeyTimestampMap newSmootherIndices_;

        // Lifecycle management & state estimation
        SystemState systemState_;
        uint32_t keyframeCounter_;

        /// @brief Mutex for accessing the system state & buffers
        std::mutex mtxState_, mtxLidarBuffer_, mtxImuBuffer_;

        /// @brief Current estimated state (propagated OR updated)
        /// @details NOTE: This is NOT the single best estimate as it is used for IMU propagation
        gtsam::NavState w_X_curr_;
        gtsam::imuBias::ConstantBias currBias_;
        double tLastImu_;

        // Mapping data
        std::list<ScanBuffer> scanBuffer_;

        // Keyframe data
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps_;
        std::map<uint32_t, std::shared_ptr<open3d::geometry::KDTreeFlann>> submapKDTrees_;
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> keyframePoses_;
        std::map<uint32_t, double> keyframeTimestamps_;

#ifndef DISABLEVIZ
        Visualization visualizer;
#endif

        // Calibration
        /// @brief Extrinsics from IMU to LiDAR frame
        /// @details See Mid360 User Manual, p. 15, Paragraph "IMU Data"
        const gtsam::Pose3 imu_T_lidar_;
        double lidarTimeOffset_;

        // Point cluster tracking
        ClusterId clusterIdCounter_;

        // Configuration constants
        static constexpr int kSlidingWindowSize = 7;
        static constexpr double kInitTimeWindow = 2.0;
        static constexpr double kVoxelSize = 0.5;
        static constexpr double kThreshNewKeyframeDist = 0.5;
        static constexpr double kMinPointDist = 1.5;
        static constexpr double kMaxPointDist = 60.0;
        static constexpr double kKnnRadius = 0.5;
        static constexpr int kKnnMaxNeighbors = 5;
        static constexpr size_t kMinClusterSize = 5;
        static constexpr double kThreshPlaneValid = 0.1;
    };

} // namespace mapping

#endif // MAPPING_MAPPINGSYSTEM_HPP_