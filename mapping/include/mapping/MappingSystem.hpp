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

#include <mapping/types.hpp>
// Visualization with Open3D
#ifndef DISABLEVIZ
#include <Visualization.hpp>
#endif

#include <mapping/factors/PointToPlaneFactor.hpp>
#include <mapping/Config.hpp>

#include <mutex>
// NOTE: used for stopwatches, remove when done debugging
#include <chrono>

namespace mapping
{
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

        /// @brief Initialize the GTSAM fixed lag smoother with update config variables.
        /// Use this when constructing with default args and calling setConfig() later.
        void initializeSmoother(const MappingConfig &config);

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
        /// @brief "Marginalize" (i.e. remove) old keyframes and their associations with clusters.
        /// Do this **before** tracking clusters so that parameters will be updated with old points already removed.
        /// Does not touch clusters as parameters will be updated during tracking.
        /// Note that when using iSAM2 it is necessary to avoid marginalizing a factor associated with KFs within the
        /// sliding window, because that would permanently fix the linearization points of these variables, resulting
        /// in sub-par estimation accuracy as the delta increases.
        /// This behavior is explained in the
        /// [`gtsam::iSAM2::marginalizeLeaves()`](https://gtsam.org/doxygen/a04340.html#a321fb6f90eb0035ef8e5bb383f8da4a2) function,
        ///
        /// > "Marginalization leaves a linear approximation of the marginal in the system,
        /// > and the linearization points of any variables involved in this linear marginal become fixed."
        /// @param idxKeyframe Index of the current (newly created) keyframe, used to determine which keyframes to marginalize.
        void marginalizeKeyframesOutsideSlidingWindow(const uint32_t &idxKeyframe);
        /// @brief Fit a plane to a set of 3D points using SVD
        /// @param points Input points to fit the plane to
        /// @param planarityThreshold Maximum ratio σ₃/σ₂ for valid plane (default 0.3)
        /// @param linearityThreshold Minimum ratio σ₂/σ₁ to reject collinear points (default 0.1)
        /// @return Tuple of (isValid, planeNormal, planeCenter, planePoints) where:
        ///         - isValid: true if plane passes planarity and non-linearity checks
        ///         - planeNormal: fitted plane normal vector
        ///         - planeCenter: centroid of input points
        ///         - planePoints: Nx3 matrix of centered points (w_p - w_planeCenter)
        ///         - planeThickness: mean squared point-to-plane distance (lower is better)
        std::tuple<bool, Eigen::Vector3d, Eigen::Vector3d, Eigen::MatrixXd, double> planeFitSVD(
            const std::vector<Eigen::Vector3d> &points,
            double planarityThreshold = 0.1,
            double linearityThreshold = 0.5) const;

        // create new clusters from points
        void createNewClusters(const uint32_t &idxKeyframe, std::size_t sampling = 1);
        void addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness);
        /// @brief Remove a point when 6-sigma test fails.
        /// Note that after this, "updateClusterParameters" still needs to be called explicitly (with recalcPlaneThickness=true).
        /// @param firstInHistory Set this to true if you are marginalizing the oldest keyframe from the cluster.
        void removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory = false);
        /// @brief Update cluster parameters from scratch
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const ClusterId &clusterId, bool recalcPlaneThickness);
        /// @brief Set new cluster parameters including thickness (i.e. valid tracking and point was added)
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const ClusterId &clusterId, const Eigen::Vector3d &newNormal, const Eigen::Vector3d &newCenter);
        void removeKeyframeFromClusters(const uint32_t &idxKeyframe);
        void pruneClusters(const uint32_t &idxKeyframe);
        // Shorthand check for whether a cluster is valid (= tracking or idle).
        bool isClusterValid(const ClusterId &clusterId) const
        {
            return !(clusterStates_.at(clusterId) == ClusterState::Premature || clusterStates_.at(clusterId) == ClusterState::Pruned);
        };
        /// @brief Summarize current clusters for debugging purposes.
        void summarizeClusters() const;
        /// @brief Summarize current factors for debugging purposes.
        void summarizeFactors() const;
        /// @brief Create new factors for previously unttracked clusters and update existing factors.
        /// **Warning:** re-keying (i.e. modifying factor-key associations in-place) is not tracked by
        /// GTSAM, i.e. the factor graph is treated as static after initialization for the solver.
        /// While iSAM2 does keep state (bayes-tree) to track which variables need to be relinearized,
        /// it does not re-key internally ([see also](https://github.com/borglab/gtsam/issues/149)).
        /// This means that updates to factor-key associations can only be passed through copy-add-remove.
        void createAndUpdateFactors(const uint32_t &idxKeyframe);
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
        gtsam::Values smootherEstimate_;

        // Factors, nodes and timestamps to add to the graph
        gtsam::NonlinearFactorGraph newSmootherFactors_;
        gtsam::Values newValues_;
        gtsam::FixedLagSmoother::KeyTimestampMap newSmootherIndices_;

        // Robust kernel for point-to-plane factors
        // see also: https://gtsam.org/doxygen/a03860.html
        gtsam::noiseModel::mEstimator::GemanMcClure::shared_ptr kernel_;

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
        std::map<ClusterId, std::map<uint32_t, std::size_t>> clusters_;
        /// @brief The thickness of each added cluster track
        std::map<ClusterId, std::vector<double>> clusterPlaneThicknessHistory_;
        /// @brief The thickness of a clusters fitted plane, used for validation and modelling noise characteristics.
        std::map<ClusterId, double> clusterPlaneThickness_;
        std::map<ClusterId, double> clusterSigmas_;
        std::map<ClusterId, ClusterState> clusterStates_;
        /// @brief Cached cluster centroids and normals for fast access during tracking and formulating smoothing constraints.
        std::map<ClusterId, std::shared_ptr<Eigen::Vector3d>> clusterCenters_, clusterNormals_;
        size_t scansSinceLastKeyframe_ = 0;

        std::map<ClusterId, boost::shared_ptr<PointToPlaneFactor>> clusterFactors_;
        gtsam::FactorIndices factorsToRemove_;

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
