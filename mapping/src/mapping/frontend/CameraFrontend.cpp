#include <mapping/frontend/CameraFrontend.hpp>

namespace mapping
{
    void CameraFrontend::setColorSpace(mapping::ColorSpace colorSpace)
    {
        _colorSpace = colorSpace;
    }

    void CameraFrontend::setCalibration(
        mapping::CameraCalibrationType calibrationType,
        double fx,
        double fy,
        double cx,
        double cy,
        std::vector<float> distortionCoefficients = {})
    {
        _calibrationType = calibrationType;
        _pinholeParams = {fx, fy, cx, cy};
        _distortionCoefficients = distortionCoefficients;
        _hasCalibration = true;
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