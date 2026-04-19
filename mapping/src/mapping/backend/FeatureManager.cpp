/// @file
/// @ingroup backend_features
#include <mapping/backend/FeatureManager.hpp>
#include <mapping/helpers.hpp> // planeFitSVD

#include <algorithm> // std::sort, std::min
#include <tuple>     // std::make_tuple

namespace mapping
{

    void FeatureManager::reset()
    {
        clusters_.clear();
        clusterPlaneThicknessHistory_.clear();
        clusterPlaneThickness_.clear();
        clusterSigmas_.clear();
        clusterStates_.clear();
        clusterCenters_.clear();
        clusterNormals_.clear();
        clusterFactors_.clear();
        clusterIdCounter_ = 0;
        newSmootherFactors_ = gtsam::NonlinearFactorGraph();
        factorsToRemove_.clear();
        dtKey_ = boost::none;
        extrinsicKey_ = boost::none;
    }

    void FeatureManager::setCalibrationKeys(boost::optional<gtsam::Key> dtKey, boost::optional<gtsam::Key> extrinsicKey)
    {
        dtKey_ = dtKey;
        extrinsicKey_ = extrinsicKey;
    }

    void FeatureManager::createNewClusters(const States &states, const uint32_t &idxKeyframe, double voxelSize)
    {
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &keyframeSubmaps = states.getKeyframeSubmaps();
        std::size_t numCreated = 0;
        if (voxelSize <= 0.01) // use all points for creating new clusters
        {
            for (std::size_t idxPoint = 0; idxPoint < keyframeSubmaps.at(idxKeyframe)->points_.size(); idxPoint++)
            {
                std::map<uint32_t, std::size_t> newCluster({{idxKeyframe, idxPoint}});
                auto const clusterId = clusterIdCounter_++;
                clusters_.emplace(clusterId, newCluster);
                clusterStates_.emplace(clusterId, ClusterState::Premature);
                clusterCenters_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(keyframeSubmaps.at(idxKeyframe)->points_[idxPoint]));
                clusterNormals_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero())); // safe guard, will yield zero-residuals
                numCreated++;
            }
        }
        else // use downsampling (but doesn't store the pcd) to create new clusters
        {
            const std::shared_ptr<open3d::geometry::PointCloud> &submap = keyframeSubmaps.at(idxKeyframe);
            // according to source code: (output, cubic_id, original_indices)
            auto const [_, __, voxelizedIndices] = submap->VoxelDownSampleAndTrace(
                voxelSize,
                submap->GetMinBound(),
                submap->GetMaxBound());

            for (const auto &idxsVoxelPts : voxelizedIndices)
            {
                std::size_t idxPoint = idxsVoxelPts.front(); // use first point in voxel for cluster creation
                std::map<uint32_t, std::size_t> newCluster({{idxKeyframe, idxPoint}});
                auto const clusterId = clusterIdCounter_++;
                clusters_.emplace(clusterId, newCluster);
                clusterStates_.emplace(clusterId, ClusterState::Premature);
                clusterCenters_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(keyframeSubmaps[idxKeyframe]->points_[idxPoint]));
                clusterNormals_.emplace(clusterId, std::make_shared<Eigen::Vector3d>(Eigen::Vector3d::Zero())); // safe guard, will yield zero-residuals
                numCreated++;
            }
        }
        std::cout << "::: [INFO] Created " << numCreated << " new clusters from keyframe " << idxKeyframe << " :::" << std::endl;
    }

    void FeatureManager::pruneClusters(const uint32_t &idxKeyframe)
    {
        std::set<ClusterId> clustersToErase;
        for (const auto &[clusterId, clusterPoints] : clusters_)
            // erase if cluster has enough points but is invalid
            if (clusterStates_.at(clusterId) == ClusterState::Pruned)
                clustersToErase.insert(clusterId);

        for (const auto &clusterId : clustersToErase)
        {
            clusters_.erase(clusterId);
            clusterStates_.erase(clusterId);
            clusterCenters_.erase(clusterId);
            clusterNormals_.erase(clusterId);
            clusterPlaneThickness_.erase(clusterId);
            clusterSigmas_.erase(clusterId);
            clusterPlaneThicknessHistory_.erase(clusterId);
            clusterFactors_.erase(clusterId);
        }
        std::cout << "::: [INFO] Pruned " << clustersToErase.size() << " clusters, "
                  << clusters_.size() << " clusters remain :::" << std::endl;
    }

    void FeatureManager::addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness)
    {
        auto const &[idxSubmap, idxPoint] = pointIdx; // destructure to make it clearer what's going on
        clusters_[clusterId][idxSubmap] = idxPoint;
        clusterPlaneThicknessHistory_[clusterId].push_back(planeThickness);
    }

    std::list<ClusterId> FeatureManager::findClustersAssociatedWithKeyframe(const uint32_t &idxKeyframe) const
    {
        std::list<ClusterId> associatedClusters;
        for (const auto &[clusterId, keyframeAssociations] : clusters_)
        {
            if (keyframeAssociations.find(idxKeyframe) != keyframeAssociations.end())
                associatedClusters.push_back(clusterId);
        }
        return associatedClusters;
    }

    void FeatureManager::removeKeyframeFromClusters(const uint32_t &idxKeyframe, const gtsam::Values &markovBlanket)
    {
        std::size_t numMarginalized = 0;
        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            auto itPoint = clusterPoints.find(idxKeyframe);
            if (itPoint != clusterPoints.end())
            {
                // --- marginalization ---
                auto existingFactorIt = clusterFactors_.find(clusterId);
                if (existingFactorIt != clusterFactors_.end())
                {
                    auto factor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactorIt->second);
                    if (factor)
                    {
                        gtsam::LinearContainerFactor::shared_ptr marginalizationFactor = factor->createMarginalizationFactor(markovBlanket, X(idxKeyframe));
                        newSmootherFactors_.add(marginalizationFactor);
                        numMarginalized++;
                    }
                }
                removePointFromCluster(clusterId, idxKeyframe, /*firstInHistory=*/true); // remove point and thickness history entry
                if (clusterPoints.size() < 3)                                            // NOTE: 3 points as the minimum to fit a plane
                {
                    // NOTE: Do NOT call factor->remove() here, since the smoother won't re-key factors
                    clusterStates_[clusterId] = ClusterState::Pruned;
                }
                else if (clusterStates_[clusterId] == ClusterState::Idle)
                {
                    // shift cluster associations, means that the entire factor needs to be replaced
                    // to reflect the removed keyframe association
                    clusterStates_[clusterId] = ClusterState::ShiftedIdle;
                }
            }
        }
        std::cout << "::: [INFO] Created " << numMarginalized << " marginalization factors for keyframe " << idxKeyframe << " :::" << std::endl;
    }

    void FeatureManager::removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory)
    {
        { // remove keyframe point from cluster
            auto it = clusters_[clusterId].find(idxKeyframe);
            if (it != clusters_[clusterId].end())
                clusters_[clusterId].erase(it);
        }
        { // remove thickness entry
            auto it = clusterPlaneThicknessHistory_.find(clusterId);
            // 2nd: vector<double> cluster thickness history (last entry should be latest keyframe)
            if (it != clusterPlaneThicknessHistory_.end() && !it->second.empty())
            {
                if (firstInHistory)
                    it->second.erase(it->second.begin());
                else
                    it->second.pop_back();
            }
        }
    }

    void FeatureManager::updateClusterParameters(const States &states, const ClusterId &clusterId, bool recalcPlaneThickness, const MappingConfig &config)
    {
        std::vector<Eigen::Vector3d> clusterPoints;
        clusterPoints.reserve(clusters_[clusterId].size());
        for (auto const &[idxSubmap, idxPoint] : clusters_.at(clusterId))
        {
            clusterPoints.push_back(states.getKeyframeSubmaps().at(idxSubmap)->points_[idxPoint]);
        }
        const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat, planeThickness] = planeFitSVD(
            clusterPoints, config.lidar_frontend.planarity_check.planarity, config.lidar_frontend.planarity_check.linearity);
        *clusterCenters_[clusterId] = clusterCenter;
        *clusterNormals_[clusterId] = planeNormal;
        // explicitly recalculate plane thickness when a point was added or removed
        if (recalcPlaneThickness)
        {
            double planeThicknessCovariance = 0.0;
            for (const double &thickness : clusterPlaneThicknessHistory_[clusterId])
                planeThicknessCovariance += std::pow(thickness, 2.0);
            planeThicknessCovariance /= static_cast<double>(clusterPlaneThicknessHistory_[clusterId].size());
            clusterPlaneThickness_[clusterId] = planeThicknessCovariance;
            // offset avoids near-zero variance for perfectly planar surfaces (numerical stability)
            clusterSigmas_[clusterId] = planeThicknessCovariance + 1e-4;
        }
    }

    void FeatureManager::updateClusterParameters(
        const States &states,
        const ClusterId &clusterId,
        const Eigen::Vector3d &planeNormal,
        const Eigen::Vector3d &clusterCenter)
    {
        *clusterCenters_[clusterId] = clusterCenter;
        *clusterNormals_[clusterId] = planeNormal;
        double planeThicknessCovariance = 0.0;
        for (const double &thickness : clusterPlaneThicknessHistory_[clusterId])
            planeThicknessCovariance += std::pow(thickness, 2.0);
        planeThicknessCovariance /= static_cast<double>(clusterPlaneThicknessHistory_[clusterId].size());
        clusterPlaneThickness_[clusterId] = planeThicknessCovariance;
        // offset avoids near-zero variance for perfectly planar surfaces (numerical stability)
        clusterSigmas_[clusterId] = planeThicknessCovariance + 1e-4;
    }

    std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> FeatureManager::computeTemporalCalibrationTwists(
        const States &states)
    {
        const std::map<uint32_t, double> &keyframeTimestamps = states.getKeyframeTimestamps();
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> &imuPoses = states.getKeyframeImuPoses();
        std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyframeTwists;

        /**
         * NOTE: currently time deltas are computed in LiDAR time
         * - it's the most accessible given how keyframe timestamps are stored (LiDAR time)
         * - converting between IMU and LiDAR time needs the calibration, but this is not
         *   a good idea since this twist will be used to estimate that in the first place
         */

        // twists between keyframes existing in the sliding window
        for (auto itKf = imuPoses.begin(), itKfNext = std::next(imuPoses.begin()); itKfNext != imuPoses.end(); ++itKf, ++itKfNext)
        {
            const uint32_t idxKf = itKf->first, idxKfNext = itKfNext->first;
            const gtsam::Pose3 &poseCurr = *itKf->second;
            const gtsam::Pose3 &poseNext = *itKfNext->second;
            // compute twist between consecutive keyframes
            const gtsam::Pose3 deltaPose = poseCurr.between(poseNext);
            const gtsam::Vector6 deltaVec = gtsam::Pose3::Logmap(deltaPose);
            // time delta between KFs + numerical offset to avoid branching
            const double dt = keyframeTimestamps.at(idxKfNext) - keyframeTimestamps.at(idxKf) + 1e-6;
            if (dt > 0)
                // (angVel, linVel)
                keyframeTwists[X(idxKf)] = {deltaVec.head<3>() / dt, deltaVec.tail<3>() / dt};
        }
        // the twist between the last sliding window pose and the current predicted pose
        {
            const uint32_t &idxKfCurr = states.getLatestKeyframeIdx();
            const gtsam::Pose3 &
                w_T_i_last = *imuPoses.rbegin()->second,
               w_T_i_pred = states.getCurrentState().pose(),
               dT_pred = w_T_i_last.between(w_T_i_pred);
            const gtsam::Vector6 dT_pred_vec = gtsam::Pose3::Logmap(dT_pred);
            // time delta between last KF in the window and the predicted state
            const double dt_pred = states.tLastScan_ - keyframeTimestamps.rbegin()->second + 1e-6;
            // (angVel, linVel)
            keyframeTwists[X(idxKfCurr)] = {dT_pred_vec.head<3>() / dt_pred, dT_pred_vec.tail<3>() / dt_pred};
        }
        return keyframeTwists;
    }

    std::pair<gtsam::NonlinearFactorGraph, gtsam::FactorIndices>
    FeatureManager::createAndUpdateFactors(
        const States &states,
        const gtsam::NonlinearFactorGraph &currentSmootherFactors)
    {
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> &keyframePoses = states.getKeyframePoses();
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &keyframeSubmaps = states.getKeyframeSubmaps();
        std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyframeTwists = computeTemporalCalibrationTwists(states);
        std::size_t numFactorsAdded{0}, numFactorsUpdated{0}, numFactorsRemoved{0};

        /* Build a clusterId -> smoother factor index map in a single pass.
         * Used in the branches below to find the previously-submitted factor
         * for a cluster so it can be invalidated and queued for removal.
         *
         * NOTE: previous impl did a full linear scan of the factor graph per cluster
         * with a dynamic_pointer_cast at every step — O(N_clusters * M_factors) per
         * keyframe. Pre-indexing collapses this to O(M_factors + N_clusters).
         */
        std::map<uint32_t, size_t> smootherFactorByCluster;
        for (size_t i = 0; i < currentSmootherFactors.size(); ++i)
        {
            const auto ptp = boost::dynamic_pointer_cast<PointToPlaneFactor>(currentSmootherFactors[i]);
            if (ptp)
                smootherFactorByCluster.emplace(ptp->clusterId_, i);
        }

        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            const ClusterState clusterState = clusterStates_.at(clusterId);
            // skip premature clusters, but pruned clusters need to be handled explicitly
            if (clusterState == ClusterState::Premature)
                continue;
            auto existingFactorIt = clusterFactors_.find(clusterId);
            // decide what to do based on the cluster state
            switch (clusterState)
            {
            case ClusterState::Tracked:
            {
                // cluster parameters from new track
                const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
                const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
                // add sensor-noise floor to the plane-fit variance for the factor noise model.
                // kept out of clusterSigmas_ so the front-end gating test remains unaffected.
                const double adaptiveSigma = clusterSigmas_[clusterId] + SENSOR_VARIANCE;
                if (existingFactorIt == clusterFactors_.end()) // factor does not exist, create
                {

                    // 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
                    gtsam::KeyVector keys;
                    keys.reserve(clusterPoints.size());
                    // 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
                    std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                    for (auto const &[idxKeyframe, idxPoint] : clusterPoints)
                    {
                        const gtsam::Key key = X(idxKeyframe);
                        keys.push_back(key);
                        // scan points are passed to factor in world frame
                        lidar_points[key] = keyframePoses[idxKeyframe]->transformTo(keyframeSubmaps[idxKeyframe]->points_[idxPoint]);
                    }
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Variance(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto factor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        states.getImuToLidarExtrinsic(),
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId,
                        dtKey_,
                        extrinsicKey_,
                        keyframeTwists);
                    newSmootherFactors_.add(factor);
                    // factor->print();
                    clusterFactors_[clusterId] = factor;
                    numFactorsAdded++;
                }
                else // cluster factor exists, remove old and readd with updated keys
                {
                    // 1: Remove old factor from smoother (if still present)
                    auto it = smootherFactorByCluster.find(clusterId);
                    if (it != smootherFactorByCluster.end())
                    {
                        const size_t factorKey = it->second;
                        const auto existingPtpFactor =
                            boost::dynamic_pointer_cast<PointToPlaneFactor>(currentSmootherFactors[factorKey]);
                        existingPtpFactor->markInvalid();
                        factorsToRemove_.push_back(factorKey);
                        numFactorsRemoved++;
                    }
                    // 2: Create new factor from ALL current clusterPoints
                    gtsam::KeyVector keys;
                    keys.reserve(clusterPoints.size());
                    std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                    for (auto const &[kfIdx, ptIdx] : clusterPoints)
                    {
                        const gtsam::Key key = X(kfIdx);
                        keys.push_back(key);
                        lidar_points[key] = keyframePoses[kfIdx]->transformTo(keyframeSubmaps[kfIdx]->points_[ptIdx]);
                    }
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Variance(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        states.getImuToLidarExtrinsic(),
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId,
                        dtKey_,
                        extrinsicKey_,
                        keyframeTwists);
                    newSmootherFactors_.add(newFactor);
                    clusterFactors_[clusterId] = newFactor;
                    // std::cout << "::: [DEBUG] replaced factor for cluster " << clusterId << " with " << keys.size() << " keys :::" << std::endl;
                    // newFactor->print();
                    numFactorsUpdated++;
                }
            }
            break;
            case ClusterState::Idle: // key associations did not change but new state estimates cause updated plane parameters
            {
                if (existingFactorIt != clusterFactors_.end())
                {
                    boost::shared_ptr<PointToPlaneFactor> factor = existingFactorIt->second;
                    // NOTE: when (re-) creating the factor, sensor noise must be re added (equivalent to "J S Jt + P" in MSCKF)
                    const double adaptiveSigma = clusterSigmas_[clusterId] + SENSOR_VARIANCE;
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Variance(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    factor->updatePlaneParameters(clusterNormals_[clusterId], clusterCenters_[clusterId], robustNoise);
                }
            }
            break;
            case ClusterState::ShiftedIdle: // key associations changed but cluster is idle, so plane parameters are not updated
            {
                // TODO: is the same as "tracked" case, just no association to the latest keyframe?
                if (existingFactorIt == clusterFactors_.end())
                {
                    std::cout << "::: [WARN] attempting to shift factor keys of idle cluster, but factor does not exist :::" << std::endl;
                    continue; // should not happen, but safe guard
                }
                const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters_[clusterId];
                const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals_[clusterId];
                // NOTE: when (re-) creating the factor, sensor noise must be re added (equivalent to "J S Jt + P" in MSCKF)
                const double adaptiveSigma = clusterSigmas_[clusterId] + SENSOR_VARIANCE;
                // 1: Remove old factor from smoother (if still present)
                auto it = smootherFactorByCluster.find(clusterId);
                if (it != smootherFactorByCluster.end())
                {
                    const size_t factorKey = it->second;
                    const auto existingPtpFactor =
                        boost::dynamic_pointer_cast<PointToPlaneFactor>(currentSmootherFactors[factorKey]);
                    existingPtpFactor->markInvalid();
                    factorsToRemove_.push_back(factorKey);
                    numFactorsRemoved++;
                }
                // 2: Create new factor from ALL current clusterPoints
                gtsam::KeyVector keys;
                keys.reserve(clusterPoints.size());
                std::map<gtsam::Key, Eigen::Vector3d> lidar_points;
                for (auto const &[kfIdx, ptIdx] : clusterPoints)
                {
                    const gtsam::Key key = X(kfIdx);
                    keys.push_back(key);
                    lidar_points[key] = keyframePoses[kfIdx]->transformTo(keyframeSubmaps[kfIdx]->points_[ptIdx]);
                }
                const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Variance(1, adaptiveSigma);
                auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                    keys,
                    states.getImuToLidarExtrinsic(),
                    lidar_points,
                    clusterNormal,
                    clusterCenter,
                    robustNoise,
                    clusterId,
                    dtKey_,
                    extrinsicKey_,
                    keyframeTwists);
                newSmootherFactors_.add(newFactor);
                clusterFactors_[clusterId] = newFactor;
                numFactorsUpdated++;
                clusterStates_[clusterId] = ClusterState::Idle; // after shifting, cluster goes back to idle state
            }
            break;
            case ClusterState::Pruned: // factor must be removed from the smoother
            {
                auto it = smootherFactorByCluster.find(clusterId);
                if (it != smootherFactorByCluster.end())
                {
                    const size_t factorKey = it->second;
                    const auto existingPtpFactor =
                        boost::dynamic_pointer_cast<PointToPlaneFactor>(currentSmootherFactors[factorKey]);
                    existingPtpFactor->markInvalid(); // mark factor as invalid to avoid further optimization
                    // mark existing factor for removal
                    factorsToRemove_.push_back(factorKey);
                    numFactorsRemoved++;
                }
            }
            break;
            default:
                // should not reach here due to earlier checks
                continue;
            }
        }
        std::cout << "::: [INFO] adding " << numFactorsAdded
                  << " LiDAR factors, updating " << numFactorsUpdated
                  << " removing " << numFactorsRemoved << " :::" << std::endl;

        // return values of the fields while simultaneously clearing them.
        return {std::exchange(newSmootherFactors_, {}), std::exchange(factorsToRemove_, {})};
    }

    void FeatureManager::summarizeClusters() const
    {
        // collect sigmas and thickness values across valid clusters to inspect
        // the dynamic range of the noise model at runtime
        std::vector<double> sigmas, thicknesses;
        std::size_t numTracked = 0, numIdle = 0, numShifted = 0;
        sigmas.reserve(clusters_.size());
        thicknesses.reserve(clusters_.size());
        for (auto const &[clusterId, _] : clusters_)
        {
            if (!isClusterValid(clusterId))
                continue;
            sigmas.push_back(clusterSigmas_.at(clusterId));
            thicknesses.push_back(clusterPlaneThickness_.at(clusterId));
            switch (clusterStates_.at(clusterId))
            {
            case ClusterState::Tracked:
                numTracked++;
            break;
            case ClusterState::Idle:
                numIdle++;
            break;
            case ClusterState::ShiftedIdle:
                numShifted++;
            break;
            }
        }
        // min / median / 95-percentile / max from an unsorted vector, sort happens in-place
        auto stats = [](std::vector<double> &v)
        {
            std::sort(v.begin(), v.end());
            const std::size_t n = v.size();
            const std::size_t idxP95 = std::min(n - 1, static_cast<std::size_t>(0.95 * n));
            return std::make_tuple(v.front(), v[n / 2], v[idxP95], v.back());
        };
        auto const [sMin, sMed, sP95, sMax] = stats(sigmas);
        auto const [tMin, tMed, tP95, tMax] = stats(thicknesses);
        std::cout << "::: [DEBUG] cluster summary (valid=" << sigmas.size()
                  << " tracked=" << numTracked << " idle=" << numIdle << " shifted=" << numShifted << ") :::\n"
                  << "\tsigmas    [m^4]: min=" << sMin << " median=" << sMed << " p95=" << sP95 << " max=" << sMax << "\n"
                  << "\tthickness [m^2]: min=" << tMin << " median=" << tMed << " p95=" << tP95 << " max=" << tMax << std::endl;
    }

    void FeatureManager::summarizeFactors(const gtsam::NonlinearFactorGraph &factors) const
    {
        size_t numImuFactors{0}, numLidarFactors{0};
        for (size_t factorKey = 0; factorKey < factors.size(); ++factorKey)
        {
            const gtsam::NonlinearFactor::shared_ptr factor = factors[factorKey];
            const auto imuFactor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
            if (imuFactor)
            {
                numImuFactors++;
                continue;
            }
            const auto ptpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(factor);
            if (ptpFactor)
            {
                numLidarFactors++;
                continue;
            }
        }
        std::cout << "::: [DEBUG] smoother has " << numImuFactors << " IMU factors, " << numLidarFactors << " LiDAR factors." << std::endl;
    }

} // namespace mapping
