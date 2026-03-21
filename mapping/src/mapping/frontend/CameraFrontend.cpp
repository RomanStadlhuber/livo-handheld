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
        const CameraData &cameraData)
    {
        // convert color space if it doesn't match the frontend's configured color space
        cv::Mat img;
        if (cameraData.colorSpace != colorSpace_)
        {
            // only RGB <-> BGR conversions are supported
            cv::cvtColor(cameraData.img, img, cv::COLOR_BGR2RGB);
        }
        else
        {
            img = cameraData.img;
        }

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
        auto &lidarBuffer = buffers.getLidarBuffer();
        auto &cameraBuffer = buffers.getCameraBuffer();

        if (lidarBuffer.empty() || cameraBuffer.empty())
            return;

        const double tNow = lidarBuffer.rbegin()->first;
        const double keepalive = config.camera_frontend.keepalive_window;
        const double tolerance = config.camera_frontend.sync_tolerance;

        // discard images older than (tNow - keepalive_window), but keep images newer than tNow
        for (auto it = cameraBuffer.begin(); it != cameraBuffer.end();)
        {
            if (it->first < tNow - keepalive)
                it = cameraBuffer.erase(it);
            else
                ++it;
        }

        if (cameraBuffer.empty())
            return;

        // for each LiDAR scan without a synced image, find the temporally closest camera image
        for (auto &[tLidar, lidarData] : lidarBuffer)
        {
            if (lidarData->syncedCameraData != nullptr)
                continue;

            // lower_bound gives first entry with key >= tLidar
            auto it = cameraBuffer.lower_bound(tLidar);

            double tBest;
            std::shared_ptr<CameraData> bestImg;

            if (it == cameraBuffer.end())
            {
                auto prev = std::prev(it);
                tBest = prev->first;
                bestImg = prev->second;
            }
            else if (it == cameraBuffer.begin())
            {
                tBest = it->first;
                bestImg = it->second;
            }
            else
            {
                auto prev = std::prev(it);
                if (std::abs(it->first - tLidar) <= std::abs(tLidar - prev->first))
                {
                    tBest = it->first;
                    bestImg = it->second;
                }
                else
                {
                    tBest = prev->first;
                    bestImg = prev->second;
                }
            }

            if (std::abs(tBest - tLidar) <= tolerance)
                lidarData->syncedCameraData = bestImg;
        }
    }
}