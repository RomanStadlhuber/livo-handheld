#include <mapping/frontend/CameraFrontend.hpp>

namespace mapping
{
    void CameraFrontend::setColorSpace(mapping::ColorSpace colorSpace)
    {
        colorSpace_ = colorSpace;
    }

    void CameraFrontend::setCalibration(
        mapping::CameraCalibrationType calibrationType,
        double fx,
        double fy,
        double cx,
        double cy,
        std::vector<float> distortionCoefficients = {})
    {
        calibrationType_ = calibrationType;
        pinholeParams_ = {fx, fy, cx, cy};
        distortionCoefficients_ = distortionCoefficients;
        hasCalibration_ = true;
    }

    void CameraFrontend::colorizeInPlace(
        std::shared_ptr<open3d::geometry::PointCloud> ptrPcd,
        Eigen::Isometry3d camera_T_lidar,
        cv::Mat img)
    {
        /**
         * TODO:
         * - transform LiDAR-frame point to Camera-frame
         * - project 3D point to image space
         * - if within bounds, set its color from the image
         */
    }
}