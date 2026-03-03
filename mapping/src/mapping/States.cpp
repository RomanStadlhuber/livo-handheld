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

    gtsam::Values States::getMarkovBlanketForKeyframe(const uint32_t idxMarginalize, const uint32_t &idxKeyframe) const
    {
        /**
         * NOTE: all of the states that could appear as blanket terms for marginalization factors
         * currently the factors conly constrain poses X(k), so velocities and biases aren't needed here
         *
         * The markov blanket uses the current soomther estimate, removing all states that have already been marginalized.
         * Otherwise the marginalization factor will try to access those states while there are no more point associations
         *
         * WARNING: this is NOT exact because at the time of marginalization, we don't know if a new constraint will be added,
         * but it will be very noisy anyway so it should be reasonably safe to ignore it.
         */
        gtsam::Values markovBlanket;
        for (uint32_t k = idxMarginalize; k < idxKeyframe; k++)
            markovBlanket.insert(X(k), smootherEstimate_.at(X(k)));
        return markovBlanket;
    }
}