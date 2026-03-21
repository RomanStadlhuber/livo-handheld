/// @file
/// @ingroup frontend_camera
#pragma once

#ifndef MAPPING_FRONTEND_CAMERAFRONTEND_HPP_
#define MAPPING_FRONTEND_CAMERAFRONTEND_HPP_

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <open3d/geometry/PointCloud.h>
#include <Eigen/Dense>

#include <mapping/types.hpp>
#include <mapping/Config.hpp>
#include <mapping/Buffers.hpp>
#include <mapping/States.hpp>

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
        /// @param cameraData Camera image with its color space.
        /// @note If the image color space does not match the frontend's configured color space, it will be converted.
        void colorizeInPlace(
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcd,
            Eigen::Isometry3d camera_T_lidar,
            const CameraData &cameraData);

        /// @brief Check the buffers to sync LiDAR & Camera
        ///
        /// Assuming that the buffer contains unprocessed LiDAR data, it will try to find
        /// temporal associations to images, which can be used for SLAM and colorization.
        ///
        /// NOTE: this will modify the `LidarData::syncedCameraData` attribute if possible,
        /// otherwise it will stay unchanged.
        /// @param states Used to obtain extrinsic calibration between LiDAR and camera.
        /// @param buffers Is accessed and modified based on the available camera & LiDAR data.
        /// @param config Used to control the temporal synchronization tolerance
        /// & camera buffer keepalive-time.
        /// @note Modifies `buffers` in-place!
        void syncCameraToLiDAR(
            const States &states,
            Buffers &buffers,
            const MappingConfig &config);

    private:
        /// @brief The color space used to process images.
        mapping::CameraColorSpace colorSpace_{mapping::CameraColorSpace::RGB};
        /// @brief 3x3 pinhole intrinsic matrix, built once on setCalibration.
        cv::Mat K_;
        /// @brief Distortion coefficients, stored as cv::Mat for direct use with cv::projectPoints.
        cv::Mat distCoeffs_;
        /// @brief The calibration type expressed by the coefficients.
        mapping::CameraCalibrationType calibrationType_;
        bool hasCalibration_{false};
    };
}

#endif // MAPPING_FRONTEND_CAMERAFRONTEND_HPP_