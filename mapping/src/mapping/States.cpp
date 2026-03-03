#include <mapping/States.hpp>

namespace mapping
{
    uint32_t States::createKeyframeSubmap(
        const gtsam::Pose3 &world_T_lidar,
        double keyframeTimestamp,
        std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap)
    {
        // Increment keyframe counter
        const uint32_t idxNewKf = keyframeCounter_++;

        // Transform submap to its estimated pose in the world frame
        ptrKeyframeSubmap->Transform(world_T_lidar.matrix());
        keyframeSubmaps_[idxNewKf] = ptrKeyframeSubmap;
        keyframePoses_[idxNewKf] = std::make_shared<gtsam::Pose3>(world_T_lidar);
        keyframeTimestamps_[idxNewKf] = keyframeTimestamp;
        return idxNewKf;
    }

    void States::removeKeyframe(const uint32_t idxKeyframe)
    {
        keyframeSubmaps_.erase(idxKeyframe);
        keyframePoses_.erase(idxKeyframe);
        keyframeTimestamps_.erase(idxKeyframe);
    }
}