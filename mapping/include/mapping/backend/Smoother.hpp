#pragma once

#ifndef MAPPING_BACKEND_SMOOTHER_HPP_
#define MAPPING_BACKEND_SMOOTHER_HPP_

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <mapping/Config.hpp>
#include <mapping/States.hpp>
#include <mapping/types.hpp>

#include <iostream>

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
        /// @param priors Prior factors must constrain the full SLAM state Pose `X(0)`, Velocity `V(0)`, and Bias `B(0)`.
        /// @param x0 Initial pose and velocity estimate (NavState) for the first keyframe.
        /// @param b0 Initial bias estimate for the first keyframe.
        void setPriors(
            const uint32_t &idxKeyframe,
            const gtsam::NonlinearFactorGraph & priors,
            const gtsam::NavState &x0,
            const gtsam::imuBias::ConstantBias &b0
        );

        /// @brief Basically just a wrapper around multiple iSAM2 update steps.
        /// After using this, call ImuFrontend::resetPreintegrator() to reset the preintegrator.
        ///
        /// NOTE: will internally modify states & submap pointclouds to reflect the update.
        /// @param idxKeyframe The index of the current keyframe, used for factor creation and marginalization.
        /// @param states The current system states, used to extend the graph and update the estimates.
        /// @param config The system config, used for update parameters.
        /// @param preintegrationFactor The preintegration factor to add to the graph for the current keyframe.
        /// @param newAndUpdatedFactors Feature factors to add to the graph or replace exsting ones.
        /// @param factorsToRemove The indices of the factors to remove from the graph, originating from the FeatureManager.
        void updateAndOptimizeGraph(
            const uint32_t &idxKeyframe,
            States &states, // will be modified in-place
            const MappingConfig &config,
            // NOTE: make it a separate variable to make sure optimize is always called from with IMU constraints.
            const gtsam::CombinedImuFactor &preintegrationFactor,
            // NOTE: will be modified in-place
            gtsam::NonlinearFactorGraph newAndUpdatedFactors,
            const gtsam::FactorIndices &factorsToRemove);

        /// @brief A relay to `IncrementalFixedLagSmoother::getFactors()`,
        /// which is needed by the FeatureManager for factor removal, replacements & marginalization.
        const gtsam::NonlinearFactorGraph &getFactors() const {return smoother_.getFactors();};

    private:
        // shorthand indicator on whether to add priors before the update pass
        bool hasPriors() const { return !initialPriors_.empty(); };

    private:
        /// @brief The GTSAM fixed lag smoother instance (based on iSAM2).
        gtsam::IncrementalFixedLagSmoother smoother_;
        // buffer for initial state values, prior factors and timestamps to add on the first update pass
        gtsam::NonlinearFactorGraph initialPriors_;
        gtsam::Values initialValues_;
        gtsam::FixedLagSmootherKeyTimestampMap initialSmootherIndices_;
        // whether the initial priors have been added to the graph and the smoother has been bootstrapped
        bool bootstrapped_{false};
    };
} // namespace mapping

#endif // MAPPING_BACKEND_SMOOTHER_HPP_