#pragma once

#ifndef MAPPING_VISUALIZATION_HPP_
#define MAPPING_VISUALIZATION_HPP_

#include <open3d/visualization/visualizer/Visualizer.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <Eigen/Dense>
#include <map>

namespace mapping
{
    class Visualization
    {
    public:
        Visualization();
        ~Visualization() = default;

        void createWindow(const bool &addWorldFrame = true);

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

    private:
        bool windowRunning = false;
        open3d::visualization::Visualizer o3dVisualizer;
        std::map<u_int32_t, std::shared_ptr<open3d::geometry::PointCloud>> submapPCDs;
        std::map<u_int32_t, std::shared_ptr<open3d::geometry::TriangleMesh>> submapPoseFrames;
        std::shared_ptr<open3d::geometry::TriangleMesh> worldFrame;
    };

} // namespace mapping

#endif // MAPPING_VISUALIZATION_HPP_