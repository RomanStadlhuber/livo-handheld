/// @file
/// @ingroup states
#include <mapping/States.hpp>

namespace mapping
{
    uint32_t States::createKeyframeSubmap(const gtsam::Pose3 &world_T_imu, double keyframeTimestamp,
                                          std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap)
    {
        // Increment keyframe counter
        const uint32_t idxNewKf = keyframeCounter_++;

        const gtsam::Pose3 world_T_lidar = world_T_imu.compose(imu_T_lidar_);
        // Transform submap to its estimated pose in the world frame
        ptrKeyframeSubmap->Transform(world_T_lidar.matrix());
        keyframeSubmaps_[idxNewKf] = ptrKeyframeSubmap;
        keyframePoses_[idxNewKf] = std::make_shared<gtsam::Pose3>(world_T_lidar);
        keyframeImuPoses_[idxNewKf] = std::make_shared<gtsam::Pose3>(world_T_imu);
        keyframeTimestamps_[idxNewKf] = keyframeTimestamp;
        return idxNewKf;
    }

    void States::updateKeyframeSubmapPose(uint32_t keyframeIdx, const gtsam::Pose3 &world_T_imu)
    {
        // imu_T_lidar_ is the optimised extrinsic at this point (Smoother reads it back before calling here)
        const gtsam::Pose3 world_T_lidar = world_T_imu.compose(imu_T_lidar_);
        // let D be the delta that needs to be left-applied via Open3D's transform
        // D * T1 = T2 | (...) * T1^-1
        // => D = T2 * T1^-1
        const gtsam::Pose3 deltaPose = world_T_lidar.compose(keyframePoses_[keyframeIdx]->inverse());
        keyframeSubmaps_[keyframeIdx]->Transform(deltaPose.matrix());
        keyframePoses_[keyframeIdx] = std::make_shared<gtsam::Pose3>(world_T_lidar);
        keyframeImuPoses_[keyframeIdx] = std::make_shared<gtsam::Pose3>(world_T_imu);
    }

    void States::removeKeyframe(const uint32_t idxKeyframe)
    {
        if (collectMarginalizedSubmaps_)
        {
            marginalizedSubmapClouds_[idxKeyframe] = keyframeSubmaps_[idxKeyframe];
            marginalizedSubmapPoses_[idxKeyframe] = keyframePoses_[idxKeyframe];
        }
        keyframeSubmaps_.erase(idxKeyframe);
        keyframePoses_.erase(idxKeyframe);
        keyframeImuPoses_.erase(idxKeyframe);
        keyframeTimestamps_.erase(idxKeyframe);
    }

    gtsam::Values States::getMarkovBlanketForKeyframe(const uint32_t idxMarginalize, const uint32_t &idxKeyframe) const
    {
        /**
         * NOTE: all of the states that could appear as blanket terms for marginalization factors
         * currently the factors conly constrain poses X(k), so velocities and biases aren't needed here
         *
         * The markov blanket uses the current soomther estimate, removing all states that have already been
         * marginalized. Otherwise the marginalization factor will try to access those states while there are no more
         * point associations
         *
         * WARNING: this is NOT exact because at the time of marginalization, we don't know if a new constraint will be
         * added, but it will be very noisy anyway so it should be reasonably safe to ignore it.
         */
        gtsam::Values markovBlanket;
        for (uint32_t k = idxMarginalize; k < idxKeyframe; k++)
            markovBlanket.insert(X(k), smootherEstimate_.at(X(k)));
        // Include calibration variables if they exist (factors now connect to them)
        if (smootherEstimate_.exists(E(0)))
            markovBlanket.insert(E(0), smootherEstimate_.at(E(0)));
        if (smootherEstimate_.exists(T(0)))
            markovBlanket.insert(T(0), smootherEstimate_.at(T(0)));
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
                {
                    marginalizedSubmapClouds_[idx] = ptrSubmap;
                    marginalizedSubmapPoses_[idx] = keyframePoses_.at(idx);
                }
            }
        }

        // preserve recovery keyframe data
        auto recoverySubmap = keyframeSubmaps_.at(idxRecovery);
        auto recoveryPose = keyframePoses_.at(idxRecovery);
        auto recoveryImuPose = keyframeImuPoses_.at(idxRecovery);
        double recoveryTimestamp = keyframeTimestamps_.at(idxRecovery);

        // clear all keyframe maps and re-insert only the recovery keyframe
        keyframeSubmaps_.clear();
        keyframePoses_.clear();
        keyframeImuPoses_.clear();
        keyframeTimestamps_.clear();

        keyframeSubmaps_[idxRecovery] = recoverySubmap;
        keyframePoses_[idxRecovery] = recoveryPose;
        keyframeImuPoses_[idxRecovery] = recoveryImuPose;
        keyframeTimestamps_[idxRecovery] = recoveryTimestamp;

        // reset navigation state to recovery estimate
        w_X_curr_ = recoveryState;
        w_X_preint_ = recoveryState;

        // clear stale smoother estimate (graph was reset)
        smootherEstimate_ = gtsam::Values();
    }

    std::map<uint32_t, std::pair<std::shared_ptr<gtsam::Pose3>, std::shared_ptr<open3d::geometry::PointCloud>>>
    States::getMarginalizedSubmaps()
    {
        std::map<uint32_t, std::pair<std::shared_ptr<gtsam::Pose3>, std::shared_ptr<open3d::geometry::PointCloud>>>
            result;
        for (auto &[idx, cloud] : marginalizedSubmapClouds_)
            result[idx] = {marginalizedSubmapPoses_.at(idx), cloud};
        marginalizedSubmapClouds_.clear();
        marginalizedSubmapPoses_.clear();
        return result;
    }
} // namespace mapping