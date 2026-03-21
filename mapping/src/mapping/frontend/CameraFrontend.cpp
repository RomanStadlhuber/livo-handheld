#include <mapping/frontend/CameraFrontend.hpp>

namespace mapping
{
    void CameraFrontend::setColorSpace(mapping::CameraColorSpace colorSpace)
    {
        colorSpace_ = colorSpace;
    }

    void CameraFrontend::setCalibration(
        mapping::CameraCalibrationType calibrationType,
        float fx,
        float fy,
        float cx,
        float cy,
        std::vector<float> distortionCoefficients)
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

    void CameraFrontend::syncCameraToLiDAR(
        const States &states,
        Buffers &buffers,
        const MappingConfig &config)
    {
        /**
         * TODO: synchronization logic
         * - current (latest) time is determined by newest LiDAR buffer timestamp-key
         * - images oulder than (tNow - tKeepalive) are discarded (newer than tNow are kept)
         * - for each timestamp in the LiDAR buffer, find the temporally closest image
         * - if std::abs(tImg - tLiDAR) <= config.camera_frontend.keepalive_window, set the sync association
         */
    }
}