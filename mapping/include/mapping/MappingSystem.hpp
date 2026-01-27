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
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// Visualization with Open3D
#ifndef DISABLEVIZ
#include <Visualization.hpp>
#endif

#include <mapping/factors/PointToPlaneFactor.hpp>
#include <mapping/Config.hpp>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
// NOTE: used for stopwatches, remove when done debugging
#include <chrono>

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

    /// @brief NavState with associated timestamp
    struct NavStateStamped
    {
        gtsam::NavState state;
        double timestamp;
    };

    /// @brief Shorthand wrapped representation for a tracked cluster.
    /// **only** used for external visualization and debugging.
    struct PointCluster
    {
        std::shared_ptr<Eigen::Vector3d> center;
        std::shared_ptr<Eigen::Vector3d> normal;
    };

    using SlidingWindowStates = std::map<uint32_t, NavStateStamped>;

    /// @brief Indexing of points in a submap for building point clusters
    /// @details Uses `int` for point index because that's what Open3D's KNN search returns.
    using SubmapIdxPointIdx = std::pair<uint32_t, int>;

    /// @brief Type used to identify point clusters
    using ClusterId = uint32_t;

    using ClusterTracks = std::map<SubmapIdxPointIdx, ClusterId>;
    using Clusters = std::map<ClusterId, std::vector<ClusterTracks::iterator>>;

    /// @brief Factors associated with valid clusters tracks including their index in the smoothers factor graph
    using ClusterFactors = std::map<ClusterId, std::pair<PointToPlaneFactor::shared_ptr, size_t>>;

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
        explicit MappingSystem(const MappingConfig &config);
        MappingSystem();

        ~MappingSystem() = default;

        /// @brief  Set the config after initialization.
        /// @details This should not be used to update the config during operation,
        /// but rather before starting to feed data.
        /// @param config
        void setConfig(const MappingConfig &config);

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

        /// @brief Get the timestamped states currently in the sliding window.
        SlidingWindowStates getStates() const;

        /// @brief Get the submap pointcloud of the latest keyframe.
        /// May return `nullptr` if no keyframes exist.
        std::shared_ptr<open3d::geometry::PointCloud> getCurrentSubmap() const;

        /// @brief Get a representation of the currently tracked clusters
        /// (for visualization and debugging purposes).
        std::map<ClusterId, PointCluster> getCurrentClusters() const;

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

        /// @brief Lock the IMU buffer and preintegrate all measurements within it.
        /// @return A NavState containing the preintegrated pose and velocity.
        gtsam::NavState preintegrateIMU();
        /// @brief Undistort all scans based on previous motion and store them to the pointcloud buffer.
        void undistortScans();
        /// @brief Track the current keyframe submap against persistent "same plane point" clusters
        /// @param idxKeyframe Index of the current keyframe
        /// @return True if the keyframe was tracked successfully
        bool trackScanPointsToClusters(const uint32_t &idxKeyframe);

        /// @brief Fit a plane to a set of 3D points using SVD
        /// @param points Input points to fit the plane to
        /// @param planarityThreshold Maximum ratio σ₃/σ₂ for valid plane (default 0.3)
        /// @param linearityThreshold Minimum ratio σ₂/σ₁ to reject collinear points (default 0.1)
        /// @return Tuple of (isValid, planeNormal, planeCenter, planePoints) where:
        ///         - isValid: true if plane passes planarity and non-linearity checks
        ///         - planeNormal: fitted plane normal vector
        ///         - planeCenter: centroid of input points
        ///         - planePoints: Nx3 matrix of centered points (w_p - w_planeCenter)
        std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd> planeFitSVD(
            const std::vector<Eigen::Vector3d> &points,
            double planarityThreshold = 0.3,
            double linearityThreshold = 0.1) const;
        // create new clusters from points
        void createNewClusters(const uint32_t &idxKeyframe, std::vector<SubmapIdxPointIdx> &clusterPoints);
        void addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness);
        void removeKeyframeFromClusters(const u_int32_t &idxKeyframe);
        void pruneClusters(const uint32_t &idxKeyframe);
        /// @brief Summarize current clusters for debugging purposes.
        void summarizeClusters() const;
        /// @brief Summarize current factors for debugging purposes.
        void summarizeFactors() const;
        /// @brief Create new factors for previously unttracked clusters and update existing factors.
        void createAndUpdateFactors();
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

        /// @brief Add a new (undistorted) scan pointcloud to the scan buffer (but don't apply pose transformation)
        /// @details The scan pointcloud is expected to be undistorted already, but all points lie in the scan frame.
        /// Use the scan buffer entries pose data to transform the scan to the desired frame.
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
        gtsam::IncrementalFixedLagSmoother smoother_;

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
        // LiDAR Same Plane Point Clusters
        std::map<ClusterId, std::map<u_int32_t, std::size_t>> clusters_;
        /// @brief The thickness of each added cluster track
        std::map<ClusterId, std::vector<double>> clusterPlaneThicknessHistory_;
        /// @brief The thickness of a clusters fitted plane, used for validation and modelling noise characteristics.
        std::map<ClusterId, double> clusterPlaneThickness_;
        std::map<ClusterId, double> clusterSigmas_;
        std::map<ClusterId, bool> clusterValidity_;
        /// @brief Cached cluster centroids and normals for fast access during tracking and formulating smoothing constraints.
        std::map<ClusterId, std::shared_ptr<Eigen::Vector3d>> clusterCenters_, clusterNormals_;
        gtsam::FactorIndices factorsToRemove_;
        size_t scansSinceLastKeyframe_ = 0;

        // Keyframe data
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps_;
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> keyframePoses_;
        std::map<uint32_t, double> keyframeTimestamps_;

#ifndef DISABLEVIZ
        Visualization visualizer;
#endif

        // Point cluster tracking
        ClusterId clusterIdCounter_;

        // Configuration
        MappingConfig config_;

        // Derived from config (cached for convenience)
        gtsam::Pose3 imu_T_lidar_;
        double lidarTimeOffset_;
    };

} // namespace mapping

#endif // MAPPING_MAPPINGSYSTEM_HPP_