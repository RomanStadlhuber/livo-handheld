/// @file
/// @ingroup frontend_camera
#pragma once

#ifndef MAPPING_FRONTEND_LIDARFRONTEND_HPP_
#define MAPPING_FRONTEND_LIDARFRONTEND_HPP_

#include <array>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>

#include <mapping/types.hpp>

namespace mapping
{
    /// @ingroup frontend_camera
    class CameraFrontend
    {
    public:
        /// @note The camera frontend defaults to processing and outputting RGB!
        CameraFrontend() = default;
        ~CameraFrontend() = default;

        /// @brief Change the color space within which images are processed.
        void setColorSpace(mapping::CameraColorSpace colorSpace);

        /// @brief Set the initial calibration (perspective camera).
        /// @param calibrationType The type of camera that is being used.
        /// @param fx Horizntal focal length in [px].
        /// @param fy Vertical focal length in [px].
        /// @param cx Horizontal center offset in [px].
        /// @param cy Vertical center offset in [px].
        /// @param distortionCoefficients Coefficients of the perspective distortion model.
        /// @note `PinholeRadTan` uses the default `cv` namespace, whereas
        /// `PinholeEquidistant` uses `cv::fisheye` and is **not supported as of yet**.
        void setCalibration(
            mapping::CameraCalibrationType calibrationType,
            float fx,
            float fy,
            float cx,
            float cy,
            std::vector<float> distortionCoefficients = {});

        /// @brief Colorize a pointcloud (LiDAR-frame) in-place.
        /// @param ptrPcd Shared ptr to the pointcloud (assumed in LiDAR-frame!!)
        /// @param camera_T_lidar Extrinsic calibration to express a LiDAR point in the camera frame.
        /// @param img Camera image.
        void colorizeInPlace(
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcd,
            Eigen::Isometry3d camera_T_lidar,
            cv::Mat img);

    private:
        /// @brief The color space used to process images.
        mapping::CameraColorSpace colorSpace_{mapping::CameraColorSpace::RGB};
        /// @brief pinhole parameters in order `fx, fy, cx, cy`.
        std::array<float, 4U> pinholeParams_{1, 1, 0, 0};
        /// @brief Coefficients of the distortion model
        std::vector<float> distortionCoefficients_;
        /// @brief The calibration type expressed by the coefficients.
        mapping::CameraCalibrationType calibrationType_;
        bool hasCalibration_{false};
    };
}

#endif // MAPPING_FRONTEND_LIDARFRONTEND_HPP_