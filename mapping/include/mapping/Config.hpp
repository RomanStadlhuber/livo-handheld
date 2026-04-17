/// @file
/// @ingroup config
#pragma once

#ifndef MAPPING_CONFIG_HPP_
#define MAPPING_CONFIG_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>

namespace mapping
{
    /// @ingroup config
    /// @brief Clustering parameters for point cluster validation
    struct ClusteringConfig
    {
        size_t min_points = 5;
        size_t max_points = 20;
        double max_plane_thickness = 0.1;          // [m]
        double sampling_voxel_size = 2.5;          // [m], voxel size to use when supplementing new clusters
        uint32_t insert_lag = 2;                   // keyframe delay to use when inserting new clusters from keyframes
        double normal_consistency_threshold = 0.9; // [0-1], accepting new points based updated normal consistency
    };

    inline void declare_config(ClusteringConfig &config)
    {
        using namespace config;
        name("ClusteringConfig");
        field(config.min_points, "min_points");
        field(config.max_points, "max_points");
        field(config.max_plane_thickness, "max_plane_thickness", "m");
        field(config.sampling_voxel_size, "sampling_voxel_size", "m");
        field(config.insert_lag, "insert_lag", "keyframes");
        field(config.normal_consistency_threshold, "normal_consistency_threshold");
        check(config.min_points, GT, 0, "min_points");
        check(config.max_points, GT, config.min_points, "max_points");
        check(config.max_plane_thickness, GT, 0.0, "max_plane_thickness");
        check(config.sampling_voxel_size, GT, 0.0, "sampling_voxel_size");
        check(config.insert_lag, GE, 0, "insert_lag");
        check(config.normal_consistency_threshold, GT, 0.0, "normal_consistency_threshold");
        check(config.normal_consistency_threshold, LT, 1.0, "normal_consistency_threshold");
    }

    /// @brief Keyframe creation thresholds
    struct KeyframeConfig
    {
        double thresh_distance = 0.075; // [m]
        double thresh_angle = 0.087;    // [rad], ~5 degrees
        size_t thresh_elapsed_scans = 5;
    };

    inline void declare_config(KeyframeConfig &config)
    {
        using namespace config;
        name("KeyframeConfig");
        field(config.thresh_distance, "thresh_distance", "m");
        field(config.thresh_angle, "thresh_angle", "rad");
        field(config.thresh_elapsed_scans, "thresh_elapsed_scans");
        check(config.thresh_distance, GT, 0.0, "thresh_distance");
        check(config.thresh_angle, GT, 0.0, "thresh_angle");
    }

    /// @brief LiDAR frontend processing parameters
    struct LidarFrontendConfig
    {
        double voxel_size = 2.0; // [m]
        bool undistort = true;   // disable for solid state LiDARs
        double knn_radius = 1.5; // [m]
        int knn_neighbors = 5;
        ClusteringConfig clustering;
        KeyframeConfig keyframe;
    };

    inline void declare_config(LidarFrontendConfig &config)
    {
        using namespace config;
        name("LidarFrontendConfig");
        field(config.voxel_size, "voxel_size", "m");
        field(config.undistort, "undistort");
        field(config.knn_radius, "knn_radius", "m");
        field(config.knn_neighbors, "knn_neighbors");
        field(config.clustering, "clustering");
        field(config.keyframe, "keyframe");
        check(config.voxel_size, GT, 0.0, "voxel_size");
        check(config.knn_radius, GT, 0.0, "knn_radius");
        check(config.knn_neighbors, GT, 0, "knn_neighbors");
    }

    /// @brief Backend optimization parameters
    struct BackendConfig
    {
        size_t sliding_window_size = 7; // [keyframes]
        double init_time_window = 2.0;  // [sec.]
        size_t solver_iterations = 2;
        double isam2_relinearize_threshold = 0.01; // frobenius norm for relinearization on state variables
    };

    inline void declare_config(BackendConfig &config)
    {
        using namespace config;
        name("BackendConfig");
        field(config.sliding_window_size, "sliding_window_size", "keyframes");
        field(config.init_time_window, "init_time_window", "s");
        field(config.solver_iterations, "solver_iterations");
        field(config.isam2_relinearize_threshold, "isam2_relinearize_threshold");
        check(config.sliding_window_size, GT, 0, "sliding_window_size");
        check(config.init_time_window, GT, 0.0, "init_time_window");
        check(config.solver_iterations, GT, 0, "solver_iterations");
        check(config.isam2_relinearize_threshold, GT, 0.0, "isam2_relinearize_threshold");
    }

    /// @brief Recovery frontend parameters
    struct RecoveryConfig
    {
        double voxel_size = 0.2;           // [m], voxel size to use for recovery reference point cloud
        size_t reference_window_size = 10; // [keyframes], number of keyframes to use for recovery reference
        size_t reference_lag = 2;          // keyframe delay to use for recovery reference
        int icp_iterations = 50;
        double max_correspondence_distance = 0.05; // [m], for validating point-to-plane ICP correspondences
    };

    inline void declare_config(RecoveryConfig &config)
    {
        using namespace config;
        name("RecoveryConfig");
        field(config.voxel_size, "voxel_size", "m");
        field(config.reference_window_size, "reference_window_size", "keyframes");
        field(config.reference_lag, "reference_lag", "keyframes");
        field(config.icp_iterations, "icp_iterations");
        field(config.max_correspondence_distance, "max_correspondence_distance", "m");
        check(config.voxel_size, GT, 0.0, "voxel_size");
        check(config.reference_window_size, GT, 1, "reference_window_size");
        check(config.reference_lag, GT, 0, "reference_lag");
        check(config.icp_iterations, GT, 1, "icp_iterations");
        check(config.max_correspondence_distance, GT, 0.0, "max_correspondence_distance");
    }

    /// @brief Point cloud filtering parameters
    struct PointFilterConfig
    {
        double min_distance = 2.5;   // [m]
        double max_distance = 100.0; // [m]
    };

    inline void declare_config(PointFilterConfig &config)
    {
        using namespace config;
        name("PointFilterConfig");
        field(config.min_distance, "min_distance", "m");
        field(config.max_distance, "max_distance", "m");
        check(config.min_distance, GE, 0.0, "min_distance");
        check(config.max_distance, GT, config.min_distance, "max_distance");
    }

    /// @brief Camera frontend processing parameters
    struct CameraFrontendConfig
    {
        std::string color_space = "RGB"; // "RGB" or "BGR"
        bool colorize_scans = true;
        double sync_tolerance = 0.01;  // [s], max. temporal offset for LiDAR-camera sync
        double keepalive_window = 0.6; // [s], how long images are kept before being discarded
    };

    inline void declare_config(CameraFrontendConfig &config)
    {
        using namespace config;
        name("CameraFrontendConfig");
        field(config.color_space, "color_space");
        field(config.colorize_scans, "colorize_scans");
        field(config.sync_tolerance, "sync_tolerance", "s");
        field(config.keepalive_window, "keepalive_window", "s");
        check(config.sync_tolerance, GT, 0.0, "sync_tolerance");
        check(config.keepalive_window, GT, 0.0, "keepalive_window");
    }

    /// @brief IMU-LiDAR extrinsic calibration
    struct ImuLidarTransformConfig
    {
        Eigen::Vector3d translation{-0.011, -0.02329, 0.04412}; // [m]
        Eigen::Vector4d rotation{0, 0, 0, 1};                   // hamilton quaternion (x, y, z, w)

        /// @brief Convert config to gtsam::Pose3
        gtsam::Pose3 toPose3() const
        {
            // eigen quaternion uses (w, x, y, z) constructor order
            Eigen::Quaterniond quat(rotation(3), rotation(0), rotation(1), rotation(2));
            return gtsam::Pose3(gtsam::Rot3(quat.normalized().toRotationMatrix()),
                                gtsam::Point3(translation));
        }
    };

    inline void declare_config(ImuLidarTransformConfig &config)
    {
        using namespace config;
        name("ImuLidarTransformConfig");
        field(config.translation, "translation", "m");
        field(config.rotation, "rotation");
    }

    /// @brief IMU-Camera extrinsic calibration
    struct ImuCameraTransformConfig
    {
        Eigen::Vector3d translation{0, 0, 0}; // [m]
        Eigen::Vector4d rotation{0, 0, 0, 1}; // hamilton quaternion (x, y, z, w)

        /// @brief Convert config to Eigen::Isometry3d
        Eigen::Isometry3d toIsometry3d() const
        {
            Eigen::Quaterniond quat(rotation(3), rotation(0), rotation(1), rotation(2));
            Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
            tf.linear() = quat.normalized().toRotationMatrix();
            tf.translation() = translation;
            return tf;
        }

        /// @brief Convert config to gtsam::Pose3
        gtsam::Pose3 toPose3() const
        {
            Eigen::Quaterniond quat(rotation(3), rotation(0), rotation(1), rotation(2));
            return gtsam::Pose3(gtsam::Rot3(quat.normalized().toRotationMatrix()),
                                gtsam::Point3(translation));
        }
    };

    inline void declare_config(ImuCameraTransformConfig &config)
    {
        using namespace config;
        name("ImuCameraTransformConfig");
        field(config.translation, "translation", "m");
        field(config.rotation, "rotation");
    }

    /// @brief Extrinsic calibration parameters
    struct ExtrinsicsConfig
    {
        bool temporal_calibration_enabled = false;
        bool extrinsic_calibration_enabled = false;
        ImuLidarTransformConfig imu_T_lidar;
        ImuCameraTransformConfig imu_T_camera;
        double imu_t_lidar = 0.0; // initial temporal offset [s]
    };

    inline void declare_config(ExtrinsicsConfig &config)
    {
        using namespace config;
        name("ExtrinsicsConfig");
        field(config.temporal_calibration_enabled, "temporal_calibration_enabled");
        field(config.extrinsic_calibration_enabled, "extrinsic_calibration_enabled");
        field(config.imu_T_lidar, "imu_T_lidar");
        field(config.imu_T_camera, "imu_T_camera");
        field(config.imu_t_lidar, "imu_t_lidar", "s");
    }

    /// @brief Pinhole camera projection parameters
    struct PinholeParameters
    {
        double fx = 0.0; // [px]
        double fy = 0.0; // [px]
        double cx = 0.0; // [px]
        double cy = 0.0; // [px]
    };

    inline void declare_config(PinholeParameters &config)
    {
        using namespace config;
        name("PinholeParameters");
        field(config.fx, "fx", "px");
        field(config.fy, "fy", "px");
        field(config.cx, "cx", "px");
        field(config.cy, "cy", "px");
        check(config.fx, GT, 0.0, "fx");
        check(config.fy, GT, 0.0, "fy");
        check(config.cx, GT, 0.0, "cx");
        check(config.cy, GT, 0.0, "cy");
    }

    /// @brief Camera intrinsic calibration
    /// @note `model` must be "PinholeRadTan" (RadTan distortion) or "PinholeEquidistant" (fisheye, unsupported).
    struct CameraIntrinsicsConfig
    {
        std::string model = "PinholeRadTan";
        PinholeParameters pinhole_parameters;
        std::vector<double> distortion_coefficients;
    };

    inline void declare_config(CameraIntrinsicsConfig &config)
    {
        using namespace config;
        name("CameraIntrinsicsConfig");
        field(config.model, "model");
        field(config.pinhole_parameters, "pinhole_parameters");
        field(config.distortion_coefficients, "distortion_coefficients");
    }

    /// @brief Intrinsic calibration parameters
    struct IntrinsicsConfig
    {
        CameraIntrinsicsConfig camera;
    };

    inline void declare_config(IntrinsicsConfig &config)
    {
        using namespace config;
        name("IntrinsicsConfig");
        field(config.camera, "camera");
    }

    /// @brief Main mapping system configuration
    struct MappingConfig
    {
        BackendConfig backend;
        LidarFrontendConfig lidar_frontend;
        CameraFrontendConfig camera_frontend;
        PointFilterConfig point_filter;
        ExtrinsicsConfig extrinsics;
        RecoveryConfig recovery;
        IntrinsicsConfig intrinsics;
    };

    inline void declare_config(MappingConfig &config)
    {
        using namespace config;
        name("MappingConfig");
        field(config.backend, "backend");
        field(config.lidar_frontend, "lidar_frontend");
        field(config.camera_frontend, "camera_frontend");
        field(config.point_filter, "point_filter");
        field(config.extrinsics, "extrinsics");
        field(config.recovery, "recovery");
        field(config.intrinsics, "intrinsics");
    }

} // namespace mapping

#endif // MAPPING_CONFIG_HPP_
