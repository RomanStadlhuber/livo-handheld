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
        K_ = (cv::Mat_<double>(3, 3) <<
            fx, 0,  cx,
            0,  fy, cy,
            0,  0,  1);
        distCoeffs_ = cv::Mat(distortionCoefficients, /*copyData=*/true);
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
            // only one conversion needed, as COLOR_BGR2RGB just flips 1st & 3rd channels
            cv::cvtColor(cameraData.img, img, cv::COLOR_BGR2RGB);
        }
        else
        {
            img = cameraData.img;
        }

        if (!hasCalibration_)
            return;

        const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

        const int imgWidth = img.cols;
        const int imgHeight = img.rows;

        // initialize pointcloud colors
        ptrPcd->colors_.resize(ptrPcd->points_.size(), Eigen::Vector3d::Zero());

        // transform points to camera frame, keeping only those in front of the camera
        std::vector<int> validIndices;
        std::vector<cv::Point3d> pointsCam;
        for (int i = 0; i < static_cast<int>(ptrPcd->points_.size()); ++i)
        {
            const Eigen::Vector3d pCam = camera_T_lidar * ptrPcd->points_[i];
            if (pCam.z() <= 0.0)
                continue;
            validIndices.push_back(i);
            pointsCam.emplace_back(pCam.x(), pCam.y(), pCam.z());
        }

        if (pointsCam.empty())
            return;

        // project to image plane (points are already in camera frame, so rvec/tvec are identity)
        std::vector<cv::Point2d> projected;
        cv::projectPoints(pointsCam, rvec, tvec, K_, distCoeffs_, projected);

        // sample pixel color for each point within image bounds
        for (int j = 0; j < static_cast<int>(projected.size()); ++j)
        {
            const int u = static_cast<int>(std::round(projected[j].x));
            const int v = static_cast<int>(std::round(projected[j].y));
            if (u < 0 || u >= imgWidth || v < 0 || v >= imgHeight)
                continue;
            const cv::Vec3b &pixel = img.at<cv::Vec3b>(v, u);
            ptrPcd->colors_[validIndices[j]] = Eigen::Vector3d(pixel[0], pixel[1], pixel[2]) / 255.0;
        }
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