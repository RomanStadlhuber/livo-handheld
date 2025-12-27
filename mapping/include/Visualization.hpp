#pragma once

#ifndef MAPPING_VISUALIZATION_HPP_
#define MAPPING_VISUALIZATION_HPP_

#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/visualization/visualizer/VisualizerWithKeyCallback.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <Eigen/Dense>
#include <iostream>
#include <map>

namespace mapping
{
    class Visualization
    {
    public:
        Visualization();
        ~Visualization() = default;

        void createWindow(const bool &addWorldFrame = true);

        void visualizeScan(const std::shared_ptr<open3d::geometry::PointCloud> &pcdScan)
        {
            o3dVisualizer.ClearGeometries();
            o3dVisualizer.AddGeometry(pcdScan);
            refreshWindow();
        }

        void addSubmap(
            const u_int32_t keyframeId,
            const Eigen::Matrix4d &pose,
            const std::shared_ptr<open3d::geometry::PointCloud> &pcdSubmap);

        void updateSubmap(
            const u_int32_t keyframeId,
            const Eigen::Matrix4d &pose
            // NOTE: pointcloud should be updated from mapping system via smart pointer
        );

        void removeSubmap(
            const u_int32_t keyframeId);

        void refreshWindow();

        /// @brief Block and refresh window until spacebar is pressed
        void waitForSpacebar();

    private:
        bool windowRunning = false;
        open3d::visualization::VisualizerWithKeyCallback o3dVisualizer;
        std::map<u_int32_t, std::shared_ptr<open3d::geometry::PointCloud>> submapPCDs;
        std::map<u_int32_t, std::shared_ptr<open3d::geometry::TriangleMesh>> submapPoseFrames;
        std::shared_ptr<open3d::geometry::TriangleMesh> worldFrame;
        // ASCII code for spacebar
        static constexpr int SPACEBAR = 32;
    };

} // namespace mapping

#endif // MAPPING_VISUALIZATION_HPP_