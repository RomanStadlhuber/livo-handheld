/// @file
/// @ingroup states
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

    void States::updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &w_T_l)
    {
        // let D be the delta that needs to be left-applied via Open3D's transform
        // D * T1 = T2 | (...) * T1^-1
        // => D = T2 * T1^-1
        const gtsam::Pose3 deltaPose = w_T_l.compose(keyframePoses_[keyframeIdx]->inverse());
        keyframeSubmaps_[keyframeIdx]->Transform(deltaPose.matrix());
        keyframePoses_[keyframeIdx] = std::make_shared<gtsam::Pose3>(w_T_l);
    }

    void States::removeKeyframe(const uint32_t idxKeyframe)
    {
        if (collectMarginalizedSubmaps_)
            marginalizedSubmaps_.push_back(keyframeSubmaps_[idxKeyframe]);
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

    void States::reset(const gtsam::NavState &recoveryState)
    {
        const uint32_t idxRecovery = keyframeCounter_ - 1;

        // archive all keyframe submaps except the recovery keyframe
        if (collectMarginalizedSubmaps_)
        {
            for (auto &[idx, ptrSubmap] : keyframeSubmaps_)
            {
                if (idx != idxRecovery)
                    marginalizedSubmaps_.push_back(ptrSubmap);
            }
        }

        // preserve recovery keyframe data
        auto recoverySubmap = keyframeSubmaps_.at(idxRecovery);
        auto recoveryPose = keyframePoses_.at(idxRecovery);
        double recoveryTimestamp = keyframeTimestamps_.at(idxRecovery);

        // clear all keyframe maps and re-insert only the recovery keyframe
        keyframeSubmaps_.clear();
        keyframePoses_.clear();
        keyframeTimestamps_.clear();

        keyframeSubmaps_[idxRecovery] = recoverySubmap;
        keyframePoses_[idxRecovery] = recoveryPose;
        keyframeTimestamps_[idxRecovery] = recoveryTimestamp;

        // reset navigation state to recovery estimate
        w_X_curr_ = recoveryState;
        w_X_preint_ = recoveryState;

        // clear stale smoother estimate (graph was reset)
        smootherEstimate_ = gtsam::Values();
    }

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> States::getMarginalizedSubmaps()
    {
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> submaps;
        std::swap(submaps, marginalizedSubmaps_);
        return submaps;
    }
}