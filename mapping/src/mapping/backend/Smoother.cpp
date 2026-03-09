/// @file
/// @ingroup backend_smoother
#include <mapping/backend/Smoother.hpp>

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
        std::cout << "Initializing smoother with fixed lag of " << config.backend.sliding_window_size << std::endl;
        smoother_ = gtsam::IncrementalFixedLagSmoother(static_cast<double>(config.backend.sliding_window_size), smootherParams);
        // clear any priors that may be left
        initialPriors_.resize(0);
        initialValues_.clear();
        initialSmootherIndices_.clear();
        bootstrapped_ = false;
    }

    void Smoother::setPriors(
        const uint32_t &idxKeyframe,
        const gtsam::NonlinearFactorGraph &priors,
        const gtsam::NavState &x0,
        const gtsam::imuBias::ConstantBias &b0)
    {
        if (bootstrapped_)
        {
            std::cerr << "::: [WARNING] Attempting to set priors after the smoother has been bootstrapped. This will not have any effect. :::" << std::endl;
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

    void Smoother::updateAndOptimizeGraph(
        const uint32_t &idxKeyframe,
        States &states,
        const MappingConfig &config,
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
        const double
            tSmootherLast{static_cast<double>(idxKeyframe - 1)},
            tSmootherCurr{static_cast<double>(idxKeyframe)};
        newSmootherIndices[X(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[V(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[B(idxKeyframe - 1)] = tSmootherLast;
        newSmootherIndices[X(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices[V(idxKeyframe)] = tSmootherCurr;
        newSmootherIndices[B(idxKeyframe)] = tSmootherCurr;
        // add priors if provided
        if (hasPriors() && !bootstrapped_)
        {
            newAndUpdatedFactors.push_back(initialPriors_);
            for (const auto &[key, timestamp] : initialSmootherIndices_)
                newSmootherIndices[key] = timestamp;
            newValues.insert(initialValues_);
            bootstrapped_ = true;
            std::cout << "::: [INFO] Graph is now bootstrapped with priors :::" << std::endl;
        }
        if (!bootstrapped_) // abort if smoother is not bootstrapped
        {
            throw std::runtime_error("::: [ERROR] Attempting to update smoother without bootstrapping, this should not happen :::");
        }
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
        states.setCurrentState(gtsam::NavState(
            states.getSmootherEstimate().at(X(idxKeyframe)).cast<gtsam::Pose3>(),
            states.getSmootherEstimate().at(V(idxKeyframe)).cast<gtsam::Vector3>()));
        states.setCurrentBias(states.getSmootherEstimate().at(B(idxKeyframe)).cast<gtsam::imuBias::ConstantBias>());
        states.setPreintegrationRefState(states.getCurrentState());
        // update the poses of all keyframe submaps
        for (auto const &[idxKf, _] : states.getKeyframePoses())
        {
            const gtsam::Pose3
                // updated IMU pose in world frame
                world_T_imu = states.getSmootherEstimate().at(X(idxKf)).cast<gtsam::Pose3>(),
                // updated lidar pose in world frame
                world_T_lidar = world_T_imu.compose(states.getImuToLidarExtrinsic());
            states.updateKeyframeSubmapPose(idxKf, world_T_lidar);
        }
    }
}