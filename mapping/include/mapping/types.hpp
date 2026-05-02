/// @file
/// @ingroup types
#pragma once

#ifndef MAPPING_TYPES_HPP_
#define MAPPING_TYPES_HPP_

#include <map>
#include <list>
#include <tuple>
#include <memory>
#include <utility>
#include <queue>
#include <mutex>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/inference/Symbol.h>
#include <open3d/geometry/PointCloud.h>
#include <opencv2/opencv.hpp>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E; // Extrinsic calibration (Pose3)
using gtsam::symbol_shorthand::T; // Temporal calibration (Vector1)
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace mapping
{
    /// @ingroup types
    /// @brief The color space within which the system should operate.
    /// Change this if you have any specific requirements for interfacing
    /// with the system.
    enum class CameraColorSpace
    {
        // The default with which the system operates.
        RGB,
        // Only used with OpenCV whenever necessary.
        BGR,
    };

    /// @ingroup types
    /// @brief Camera image data.
    struct CameraData
    {
        cv::Mat img;
        CameraColorSpace colorSpace;
    };

    /// @ingroup types
    /// @brief IMU measurement data
    struct ImuData
    {
        Eigen::Vector3d acceleration;
        Eigen::Vector3d angular_velocity;
    };

    /// @ingroup types
    /// @brief LiDAR scan data with per-point timestamps
    struct LidarData
    {
        std::vector<Eigen::Vector3d> points;
        std::vector<double> offset_times;
        /// @note Expect this to be `nullptr` most of the time!
        std::shared_ptr<CameraData> syncedCameraData;
    };

    template <typename T> using InputBuffer = std::map<double, std::shared_ptr<T>>;

    /// @ingroup types
    /// @brief Buffered scan data for keyframe creation
    struct ScanBuffer
    {
        /// @brief Pointcloud of undistorted (and voxelized) scan
        std::shared_ptr<open3d::geometry::PointCloud> pcd;
        /// @brief Pose of the scan w.r.t. the latest keyframe pose
        std::shared_ptr<gtsam::Pose3> kf_T_scan;
        /// @brief Temporally synced camera image for this scan (nullptr if no association exists)
        std::shared_ptr<CameraData> syncedCameraData{nullptr};
    };

    /// @ingroup types
    /// @brief The type of camera calibration for the `CameraFrontend` to use.
    enum class CameraCalibrationType
    {
        // OpenCV default, use this when distortion is low.
        PinholeRadTan,
        /**
         * NOTE: maybe this can be supported in the future?
         * afaik, only needs to use cv::fisheye namespace
         */
        PInholeEquidistant,
    };

    /// @ingroup types
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

    /// @ingroup types
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
        /// @brief When the cluster could not be tracked in the new keyframe, and a keyframe was marginalized from it.
        /// @details The factor must be replaced, dropping the marginalized keyframe association.
        ShiftedIdle,
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

    /// @brief The "default color", inidcating that a point was not colorized from the camera image.
    const Eigen::Vector3d NO_COLOR{Eigen::Vector3d::Zero()};

    /// @ingroup types
    /// @brief Thread-safe queue for inter-thread communication
    template <typename T> class SafeQueue
    {
    public:
        void push(T item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(item));
        }

        /// @brief Non-blocking pop attempt
        /// @return false if queue is empty, true and populates item if successful
        bool try_pop(T &item)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty())
                return false;
            item = std::move(queue_.front());
            queue_.pop();
            return true;
        }

    private:
        std::queue<T> queue_;
        mutable std::mutex mutex_;
    };

    /// @ingroup types
    /// @brief State of a global map segment
    enum class SegmentState
    {
        /// @brief Segment is accumulating submaps
        Accumulating,
        /// @brief Segment is waiting for refinement
        WaitingForRefinement,
        /// @brief Segment is waiting for alignment
        WaitingForAlignment,
        /// @brief Segment has been aligned
        Aligned,
    };

    /// @ingroup types
    /// @brief A segment of the global map containing multiple submaps
    struct GlobalMapSegment
    {
        /// @brief Unique identifier for this segment
        uint32_t id;
        /// @brief Keyframe index of the first submap in this segment
        uint32_t startIdx;
        /// @brief Map of submaps: keyframe index -> (Pose3, PointCloud)
        std::map<uint32_t, std::pair<std::shared_ptr<gtsam::Pose3>, std::shared_ptr<open3d::geometry::PointCloud>>>
            submaps;
        /// @brief Current state of this segment
        SegmentState state;
        /// @brief Merged point cloud of all submaps in this segment (nullptr until merged)
        std::shared_ptr<open3d::geometry::PointCloud> pcdMerged{nullptr};
    };
} // namespace mapping
#endif // MAPPING_TYPES_HPP_