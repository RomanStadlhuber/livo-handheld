#pragma once

#ifndef MAPPING_CONFIG_HPP_
#define MAPPING_CONFIG_HPP_

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

#include <config_utilities/config.h>
#include <config_utilities/types/eigen_matrix.h>

namespace mapping
{

    /// @brief Clustering parameters for point cluster validation
    struct ClusteringConfig
    {
        size_t min_points = 5;
        size_t max_points = 20;
        double max_plane_thickness = 0.1; // [m]
        size_t supplement_sampling = 10; // subsampling stride for supplementing new clusters
    };

    inline void declare_config(ClusteringConfig &config)
    {
        using namespace config;
        name("ClusteringConfig");
        field(config.min_points, "min_points");
        field(config.max_points, "max_points");
        field(config.max_plane_thickness, "max_plane_thickness", "m");
        field(config.supplement_sampling, "supplement_sampling");
        check(config.min_points, GT, 0, "min_points");
        check(config.max_points, GT, config.min_points, "max_points");
        check(config.max_plane_thickness, GT, 0.0, "max_plane_thickness");
        check(config.supplement_sampling, GT, 0, "supplement_sampling");
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
        int sliding_window_size = 7;   // [keyframes]
        double init_time_window = 2.0; // [sec.]
        int solver_iterations = 5; 
    };

    inline void declare_config(BackendConfig &config)
    {
        using namespace config;
        name("BackendConfig");
        field(config.sliding_window_size, "sliding_window_size", "keyframes");
        field(config.init_time_window, "init_time_window", "s");
        field(config.solver_iterations, "solver_iterations");
        check(config.sliding_window_size, GT, 0, "sliding_window_size");
        check(config.init_time_window, GT, 0.0, "init_time_window");
        check(config.solver_iterations, GT, 0, "solver_iterations");
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

    /// @brief Extrinsic calibration parameters
    struct ExtrinsicsConfig
    {
        bool temporal_calibration_enabled = false;
        ImuLidarTransformConfig imu_T_lidar;
    };

    inline void declare_config(ExtrinsicsConfig &config)
    {
        using namespace config;
        name("ExtrinsicsConfig");
        field(config.temporal_calibration_enabled, "temporal_calibration_enabled");
        field(config.imu_T_lidar, "imu_T_lidar");
    }

    /// @brief Main mapping system configuration
    struct MappingConfig
    {
        BackendConfig backend;
        LidarFrontendConfig lidar_frontend;
        PointFilterConfig point_filter;
        ExtrinsicsConfig extrinsics;
    };

    inline void declare_config(MappingConfig &config)
    {
        using namespace config;
        name("MappingConfig");
        field(config.backend, "backend");
        field(config.lidar_frontend, "lidar_frontend");
        field(config.point_filter, "point_filter");
        field(config.extrinsics, "extrinsics");
    }

} // namespace mapping

#endif // MAPPING_CONFIG_HPP_
