#pragma once

#ifndef MAPPING_TYPES_HPP_
#define MAPPING_TYPES_HPP_

#include <map>
#include <list>
#include <tuple>
#include <memory>
#include <utility>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <open3d/geometry/PointCloud.h>

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

    template <typename T>
    using InputBuffer = std::map<double, std::shared_ptr<T>>;

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

    /// @brief State of a point cluster in the tracking system.
    /// See detail descriptoin for relationship to managing factors.
    enum class ClusterState
    {
        /// @brief When the cluster does not yet contain enough points. New clusters start out at this state.
        /// @details No factor can be created yet.
        Premature,
        /// @brief When the cluster was tracked successfully in a new keyframe.
        /// @details Create a new factor and add it to the graph.
        Tracked,
        /// @brief  When the cluster could not be tracked in the new keyframe.
        /// @details A factor must already exist for this cluster. It is updated incl. the new keyframe association.
        Idle,
        /// @brief Marked for removal, e.g. when failing the 6-sigma test or marginalized out.
        /// @details Any factors associated with this cluster must be removed from the graph.
        Pruned,
    };

    using SlidingWindowStates = std::map<uint32_t, NavStateStamped>;

    /// @brief Indexing of points in a submap for building point clusters
    /// @details Uses `int` for point index because that's what Open3D's KNN search returns.
    using SubmapIdxPointIdx = std::pair<uint32_t, int>;

    /// @brief Type used to identify point clusters
    using ClusterId = uint32_t;

    /// @brief Invalid cluster ID sentinel value
    constexpr ClusterId INVALID_CLUSTER_ID = std::numeric_limits<ClusterId>::max();
}
#endif // MAPPING_TYPES_HPP_