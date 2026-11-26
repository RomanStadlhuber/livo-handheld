/// @file
/// @ingroup backend_smoother
#pragma once

#ifndef MAPPING_BACKEND_SMOOTHER_HPP_
#define MAPPING_BACKEND_SMOOTHER_HPP_

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <mapping/Config.hpp>
#include <mapping/States.hpp>
#include <mapping/types.hpp>

// NOTE: FeatureManager is used by updateAndOptimizeGraph to set the cluster->factorIdx mapping
#include <mapping/backend/FeatureManager.hpp>

#include <tbb/task_arena.h>

#include <iostream>
#include <limits>
#include <optional>

namespace mapping
{
    /// @ingroup backend_smoother
    class Smoother
    {
    public:
        Smoother() = default;
        ~Smoother() = default;

        /// @brief Initialize/reset the GTSAM fixed lag smoother with update config variables.
        void reset(const MappingConfig &config);

        /// @brief Set the inital, world-frame fixing state and priors to bootstrap the optimization.
        /// Do this with the values from SLAM initialization.
        /// @param idxKeyframe The index variables associated with the first keyframe (assumed 0 by default).
        /// @param priors Prior factors must constrain the full SLAM state Pose `X(0)`, Velocity `V(0)`, and Bias
        /// `B(0)`.
        /// @param x0 Initial pose and velocity estimate (NavState) for the first keyframe.
        /// @param b0 Initial bias estimate for the first keyframe.
        void setPriors(const uint32_t &idxKeyframe, const gtsam::NonlinearFactorGraph &priors,
                       const gtsam::NavState &x0, const gtsam::imuBias::ConstantBias &b0);

        /// @brief Basically just a wrapper around multiple iSAM2 update steps.
        /// After using this, call ImuFrontend::resetPreintegrator() to reset the preintegrator.
        ///
        /// NOTE: will internally modify states & submap pointclouds to reflect the update.
        /// @param idxKeyframe The index of the current keyframe, used for factor creation and marginalization.
        /// @param states The current system states, used to extend the graph and update the estimates.
        /// @param config The system config, used for update parameters.
        /// @param preintegrationFactor The preintegration factor to add to the graph for the current keyframe.
        /// @param newAndUpdatedFactors Feature factors to add to the graph or replace exsting ones.
        /// @param factorsToRemove The indices of the factors to remove from the graph, originating from the
        /// FeatureManager.
        void updateAndOptimizeGraph(
            const uint32_t &idxKeyframe,
            States &states,                 // will be modified in-place
            FeatureManager &featureManager, // will be modified in-place
            const MappingConfig &config,
            // NOTE: make it a separate variable to make sure optimize is always called from with IMU constraints.
            const gtsam::CombinedImuFactor &preintegrationFactor,
            // NOTE: will be modified in-place
            gtsam::NonlinearFactorGraph newAndUpdatedFactors, const gtsam::FactorIndices &factorsToRemove);

        /// @brief A relay to `IncrementalFixedLagSmoother::getFactors()`,
        /// which is needed by the FeatureManager for factor removal, replacements & marginalization.
        const gtsam::NonlinearFactorGraph &getFactors() const { return smoother_.getFactors(); };

        /// @brief Insert initial calibration values, timestamps, and priors into the smoother buffers.
        void setCalibrationPriors(const MappingConfig &config, const gtsam::Pose3 &initialExtrinsic);

        /// @brief Smoother timestamp of the keys of keyframe `kf` after the update of keyframe `idxNewest`.
        /// @details Keyframe indices are the timestamps. With block release (`blockSize > 1`), the keys of a completed
        /// block `[b * blockSize, b * blockSize + blockSize - 1]` carry the index of the newest keyframe of the block,
        /// so the smoother marginalizes the block in one update. Never newer than `idxNewest`.
        static uint32_t keyframeStamp(uint32_t kf, uint32_t idxNewest, uint32_t blockSize)
        {
            if (blockSize <= 1)
                return kf;
            const uint32_t blockEnd = (kf / blockSize) * blockSize + blockSize - 1;
            return blockEnd <= idxNewest ? blockEnd : kf;
        }

        /// @brief Whether the update of keyframe `idxKeyframe` marginalizes the keys of keyframe `kf`.
        /// @details Same rule as `IncrementalFixedLagSmoother`: stamp older than the newest stamp minus the lag.
        static bool isMarginalized(uint32_t kf, uint32_t idxKeyframe, uint32_t slidingWindowSize, uint32_t blockSize)
        {
            return keyframeStamp(kf, idxKeyframe, blockSize) + slidingWindowSize < idxKeyframe;
        }

#ifdef MAPPING_BENCHMARKING
        /// @brief Marginal covariance of a variable in the current graph (benchmark build only, for telemetry).
        gtsam::Matrix marginalCovariance(const gtsam::Key key) const { return smoother_.marginalCovariance(key); }

        /// @brief A relay to `ISAM2::getFixedVariables()`: the keys that iSAM2 never relinearizes again.
        const gtsam::KeySet &getFixedVariables() const { return smoother_.getISAM2().getFixedVariables(); }

        /// @brief The LiDAR factors that the last `updateAndOptimizeGraph` call received (benchmark build only).
        /// @details A copy of the `newAndUpdatedFactors` argument, before the IMU factor and the priors join it.
        const gtsam::NonlinearFactorGraph &getLastNewFactors() const { return lastNewFactors_; }

        /// @brief The factor indices that the last `updateAndOptimizeGraph` call had to remove (benchmark build only).
        const gtsam::FactorIndices &getLastFactorsToRemove() const { return lastFactorsToRemove_; }

        /// @brief The keyframe of the last `updateAndOptimizeGraph` call (benchmark build only).
        /// @details Empty before the first call and after `reset`. Tells if the copies above belong to a keyframe.
        const std::optional<uint32_t> &getLastUpdateKeyframe() const { return lastUpdateKeyframe_; }

        /// @brief Wall time of the last `updateAndOptimizeGraph` call `[s]`: all iSAM2 updates and the estimate.
        double getLastUpdateSeconds() const { return lastUpdateSeconds_; }
#endif

    private:
        // shorthand indicator on whether to add priors before the update pass
        bool hasPriors() const { return !initialPriors_.empty(); };

    private:
        bool temporalCalibrationEnabled_{false};
        bool extrinsicCalibrationEnabled_{false};
        /// @brief Block release size (`backend.block_release_size`, benchmark build only), 0 elsewhere.
        uint32_t blockReleaseSize_{0};

        /// @brief The GTSAM fixed lag smoother instance (based on iSAM2).
        gtsam::IncrementalFixedLagSmoother smoother_;
        // buffer for initial state values, prior factors and timestamps to add on the first update pass
        gtsam::NonlinearFactorGraph initialPriors_;
        gtsam::Values initialValues_;
        gtsam::FixedLagSmootherKeyTimestampMap initialSmootherIndices_;
        // whether the initial priors have been added to the graph and the smoother has been bootstrapped
        bool bootstrapped_{false};

        // TBB arena for all GTSAM parallel work; normal priority, sized to complement the BA arena
        std::unique_ptr<tbb::task_arena> frontendArena_;

#ifdef MAPPING_BENCHMARKING
        /// @brief Arguments of the last `updateAndOptimizeGraph` call, for the telemetry.
        gtsam::NonlinearFactorGraph lastNewFactors_;
        gtsam::FactorIndices lastFactorsToRemove_;
        std::optional<uint32_t> lastUpdateKeyframe_;
        double lastUpdateSeconds_ = std::numeric_limits<double>::quiet_NaN();
#endif
    };
} // namespace mapping

#endif // MAPPING_BACKEND_SMOOTHER_HPP_