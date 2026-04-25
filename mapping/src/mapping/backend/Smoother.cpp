/// @file
/// @ingroup backend_smoother
#include <mapping/backend/Smoother.hpp>
#include <mapping/logging.hpp>

SETUP_LOGS(INFO, "Smoother")

namespace mapping
{
    void Smoother::reset(const MappingConfig &config)
    {
        // need to reset smoother with new config
        gtsam::ISAM2Params smootherParams;
        smootherParams.optimizationParams = gtsam::ISAM2GaussNewtonParams();
        // for some guidance, see:
        // https://github.com/MIT-SPARK/Kimera-VIO/blob/master/include/kimera-vio/backend/VioBackendParams.h
        smootherParams.relinearizeThreshold = config.backend.isam2_relinearize_threshold;
        smootherParams.relinearizeSkip = 1;          // only (check) relinearize after that many update calls
        smootherParams.findUnusedFactorSlots = true; // should be enabled when using smoother
        LOG(INFO, "initializing smoother with fixed lag of " << config.backend.sliding_window_size);
        smoother_ =
            gtsam::IncrementalFixedLagSmoother(static_cast<double>(config.backend.sliding_window_size), smootherParams);
        // clear any priors that may be left
        initialPriors_.resize(0);
        initialValues_.clear();
        initialSmootherIndices_.clear();
        bootstrapped_ = false;
        temporalCalibrationEnabled_ = config.extrinsics.temporal_calibration_enabled;
        extrinsicCalibrationEnabled_ = config.extrinsics.extrinsic_calibration_enabled;
    }

    void Smoother::setPriors(const uint32_t &idxKeyframe, const gtsam::NonlinearFactorGraph &priors,
                             const gtsam::NavState &x0, const gtsam::imuBias::ConstantBias &b0)
    {
        if (bootstrapped_)
        {
            LOG(WARN,
                "Attempting to set priors after the smoother has been bootstrapped. This will not have any effect.");
            return;
        }
        initialPriors_ = priors;
        initialValues_.insert(X(idxKeyframe), x0.pose());
        initialValues_.insert(V(idxKeyframe), x0.v());
        initialValues_.insert(B(idxKeyframe), b0);
        initialSmootherIndices_[X(idxKeyframe)] = 0.0;
        initialSmootherIndices_[V(idxKeyframe)] = 0.0;
        initialSmootherIndices_[B(idxKeyframe)] = 0.0;
    }

    void Smoother::setCalibrationPriors(const MappingConfig &config, const gtsam::Pose3 &initialExtrinsic)
    {
        if (config.extrinsics.temporal_calibration_enabled)
        {
            gtsam::Vector1 dt0;
            dt0 << config.extrinsics.imu_t_lidar;
            initialValues_.insert(T(0), dt0);
            initialSmootherIndices_[T(0)] = 0.0;
            initialPriors_.addPrior(T(0), dt0, gtsam::noiseModel::Isotropic::Sigma(1, 0.01)); // 10ms sigma
        }
        if (config.extrinsics.extrinsic_calibration_enabled)
        {
            initialValues_.insert(E(0), initialExtrinsic);
            initialSmootherIndices_[E(0)] = 0.0;
            gtsam::Vector6 sigma;
            sigma << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; // rad, m
            initialPriors_.addPrior(E(0), initialExtrinsic, gtsam::noiseModel::Diagonal::Sigmas(sigma));
        }
    }

    void Smoother::updateAndOptimizeGraph(const uint32_t &idxKeyframe, States &states, const MappingConfig &config,
                                          const gtsam::CombinedImuFactor &preintegrationFactor,
                                          gtsam::NonlinearFactorGraph newAndUpdatedFactors,
                                          const gtsam::FactorIndices &factorsToRemove)
    {
        // need to manually add preintegraion factor
        newAndUpdatedFactors.add(preintegrationFactor);
        // variables that join the graph with the new keyframe
        gtsam::Values newValues;
        const gtsam::NavState &w_X_curr = states.getCurrentState();
        newValues.insert(X(idxKeyframe), w_X_curr.pose());
        newValues.insert(V(idxKeyframe), w_X_curr.v());
        newValues.insert(B(idxKeyframe), states.getCurrentBias());
        // new KF indices "timestamps" to let the smoother know where the sliding window lies
        gtsam::FixedLagSmootherKeyTimestampMap newSmootherIndices;
        const double tSmootherLast{static_cast<double>(idxKeyframe - 1)},
            tSmootherCurr{static_cast<double>(idxKeyframe)};
        newSmootherIndices[X(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[V(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[B(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[X(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices[V(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices[B(idxKeyframe)] = tSmootherCurr;
        // bump calibration timestamps to keep them alive in the sliding window
        if (temporalCalibrationEnabled_)
            newSmootherIndices[T(0)] = tSmootherCurr;
        if (extrinsicCalibrationEnabled_)
            newSmootherIndices[E(0)] = tSmootherCurr;
        // add priors if provided
        if (hasPriors() && !bootstrapped_)
        {
            newAndUpdatedFactors.push_back(initialPriors_);
            for (const auto &[key, timestamp] : initialSmootherIndices_)
                newSmootherIndices[key] = timestamp;
            newValues.insert(initialValues_);
            bootstrapped_ = true;
            LOG(INFO, "graph is now bootstrapped with priors");
        }
        if (!bootstrapped_) // abort if smoother is not bootstrapped
        {
            throw std::runtime_error(
                "::: [ERROR] Attempting to update smoother without bootstrapping, this should not happen :::");
        }
        // capture calibration values before update for delta logging
        const double dt_before = temporalCalibrationEnabled_ ? states.getTemporalOffset() : 0.0;
        const gtsam::Pose3 E_before = extrinsicCalibrationEnabled_ ? states.getImuToLidarExtrinsic() : gtsam::Pose3();
        // update estimator
        smoother_.update(newAndUpdatedFactors, newValues, newSmootherIndices, factorsToRemove);
        /**
         * NOTE: iSAM2 update performs only one GN step,
         * multiple updates to assure convergence, see also
         * - https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/IncrementalFixedLagSmoother.cpp#L128
         * - https://groups.google.com/g/gtsam-users/c/Cz2RoY3dN14/m/3Ka6clsdBgAJ
         */
        for (std::size_t updateIters = 1; updateIters < config.backend.solver_iterations; updateIters++)
            smoother_.update();
        // calculate the estimate and update the shared state container
        // currently this is just matching legacy behavior
        states.setSmootherEstimate(smoother_.calculateEstimate());
        states.setCurrentState(gtsam::NavState(states.getSmootherEstimate().at(X(idxKeyframe)).cast<gtsam::Pose3>(),
                                               states.getSmootherEstimate().at(V(idxKeyframe)).cast<gtsam::Vector3>()));
        states.setCurrentBias(states.getSmootherEstimate().at(B(idxKeyframe)).cast<gtsam::imuBias::ConstantBias>());
        states.setPreintegrationRefState(states.getCurrentState());
        // read back calibration estimates and log updates
        if (extrinsicCalibrationEnabled_)
        {
            states.setImuToLidarExtrinsic(states.getSmootherEstimate().at(E(0)).cast<gtsam::Pose3>());
            const gtsam::Pose3 &E_after = states.getImuToLidarExtrinsic();
            const gtsam::Vector6 delta = gtsam::Pose3::Logmap(E_before.between(E_after));
            LOG(INFO, "delta i_T_l [rot | trans]: [" << delta.transpose() << "]");
            LOG(INFO, "i_T_l : rotation rpy [" << E_after.rotation().rpy().transpose() << "] translation ["
                                               << E_after.translation().transpose() << "]");
        }
        if (temporalCalibrationEnabled_)
        {
            const double dt_after = states.getSmootherEstimate().at(T(0)).cast<gtsam::Vector1>()(0);
            states.setTemporalOffset(dt_after);
            LOG(INFO, "l_t_i: " << (dt_after - dt_before) << " [sec.], current value: " << dt_after << " [sec.]");
        }
        // update the poses of all keyframe submaps
        for (auto const &[idxKf, _] : states.getKeyframePoses())
        {
            const gtsam::Pose3 world_T_imu = states.getSmootherEstimate().at(X(idxKf)).cast<gtsam::Pose3>();
            states.updateKeyframeSubmapPose(idxKf, world_T_imu);
        }
    }
} // namespace mapping
