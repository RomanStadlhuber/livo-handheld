#include <Visualization.hpp>

namespace mapping
{
    Visualization::Visualization() {}

    void Visualization::createWindow(const bool &addWorldFrame)
    {
        if (windowRunning)
            return;

        o3dVisualizer.CreateVisualizerWindow("Mapping Visualization", 800, 600);
        windowRunning = true;

        if (addWorldFrame)
        {
            worldFrame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
            o3dVisualizer.AddGeometry(worldFrame);
        }
        refreshWindow();
    }

    void Visualization::addSubmap(
        const uint32_t keyframeId,
        const Eigen::Matrix4d &pose,
        const std::shared_ptr<open3d::geometry::PointCloud> &pcdSubmap)
    {
        submapPCDs[keyframeId] = pcdSubmap;

        // Create a coordinate frame for the submap pose
        auto poseFrame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5);
        poseFrame->Transform(pose);
        submapPoseFrames[keyframeId] = poseFrame;

        o3dVisualizer.AddGeometry(pcdSubmap);
        o3dVisualizer.AddGeometry(poseFrame);
        refreshWindow();
    }

    void Visualization::updateSubmap(
        const uint32_t keyframeId,
        const Eigen::Matrix4d &pose)
    {
        // Update the pose frame
        if (submapPoseFrames.find(keyframeId) != submapPoseFrames.end())
        {
            auto poseFrame = submapPoseFrames[keyframeId];
            poseFrame->Transform(pose);
            o3dVisualizer.UpdateGeometry(poseFrame);
        }
        refreshWindow();
    }

    void Visualization::removeSubmap(
        const uint32_t keyframeId)
    {
        // Remove point cloud
        if (submapPCDs.find(keyframeId) != submapPCDs.end())
        {
            o3dVisualizer.RemoveGeometry(submapPCDs[keyframeId]);
            submapPCDs.erase(keyframeId);
        }

        // Remove pose frame
        if (submapPoseFrames.find(keyframeId) != submapPoseFrames.end())
        {
            o3dVisualizer.RemoveGeometry(submapPoseFrames[keyframeId]);
            submapPoseFrames.erase(keyframeId);
        }

        refreshWindow();
    }

    void Visualization::refreshWindow()
    {
        if (windowRunning)
        {
            // o3dVisualizer.PollEvents();
            o3dVisualizer.UpdateRender();
        }
    }

    void Visualization::waitForSpacebar()
    {
        if (!windowRunning)
            return;

        bool spacePressed = false;

        // Register key callback for spacebar (key code 32)
        o3dVisualizer.RegisterKeyCallback(
            SPACEBAR,
            [&spacePressed](open3d::visualization::Visualizer *vis) -> bool
            {
            spacePressed = true;
            return true; });

        std::cout << "::: [INFO] processing halted by visualization, press [SPACE] to continue :::" << std::endl;

        // Block and refresh until spacebar is pressed
        while (!spacePressed && windowRunning)
        {
            o3dVisualizer.PollEvents();
            o3dVisualizer.UpdateRender();
        }

        // Unregister the callback
        o3dVisualizer.RegisterKeyCallback(SPACEBAR, nullptr);
    }
}
