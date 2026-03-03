#include <mapping/FeatureManager.hpp>
#include <mapping/helpers.hpp> // planeFitSVD

namespace mapping
{

    void FeatureManager::createNewClusters(const States &states, const uint32_t &idxKeyframe, double voxelSize)
    {
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps = states.getKeyframeSubmaps();
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
                if (clusterPoints.size() < 3)
                { // TODO: use min-points size or 3?
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

    void FeatureManager::updateClusterParameters(const States &states, const ClusterId &clusterId, bool recalcPlaneThickness)
    {
        std::vector<Eigen::Vector3d> clusterPoints;
        clusterPoints.reserve(clusters_[clusterId].size());
        for (auto const &[idxSubmap, idxPoint] : clusters_.at(clusterId))
        {
            clusterPoints.push_back(states.getKeyframeSubmaps().at(idxSubmap)->points_[idxPoint]);
        }
        const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat, planeThickness] = planeFitSVD(clusterPoints);
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
            clusterSigmas_[clusterId] = std::pow(0.5 * planeThicknessCovariance, 0.25);
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
        clusterSigmas_[clusterId] = std::pow(0.5 * planeThicknessCovariance, 0.25);
    }

    std::pair<gtsam::NonlinearFactorGraph, gtsam::FactorIndices>
    FeatureManager::createAndUpdateFactors(
        const States &states,
        const gtsam::NonlinearFactorGraph &currentSmootherFactors)
    {
        std::map<uint32_t, std::shared_ptr<gtsam::Pose3>> &keyframePoses = states.getKeyframePoses();
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &keyframeSubmaps = states.getKeyframeSubmaps();
        std::size_t numFactorsAdded{0}, numFactorsUpdated{0}, numFactorsRemoved{0};
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
                const double adaptiveSigma = clusterSigmas_[clusterId];
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
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto factor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        states.getImuToLidarExtrinsic(),
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId);
                    newSmootherFactors_.add(factor);
                    // factor->print();
                    clusterFactors_[clusterId] = factor;
                    numFactorsAdded++;
                }
                else // cluster factor exists, remove old and readd with updated keys
                {
                    // 1: Remove old factor from smoother (if still present)
                    const gtsam::NonlinearFactorGraph &smootherFactors = currentSmootherFactors;
                    for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                    {
                        const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                        const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                        if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                        {
                            existingPtpFactor->markInvalid();
                            factorsToRemove_.push_back(gtsam::Key{factorKey});
                            numFactorsRemoved++;
                            break;
                        }
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
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                    auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                    const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                        keys,
                        states.getImuToLidarExtrinsic(),
                        lidar_points,
                        clusterNormal,
                        clusterCenter,
                        robustNoise,
                        clusterId);
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
                    const double adaptiveSigma = clusterSigmas_[clusterId];
                    const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
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
                const double adaptiveSigma = clusterSigmas_[clusterId];
                // 1: Remove old factor from smoother (if still present)
                const gtsam::NonlinearFactorGraph &smootherFactors = currentSmootherFactors;
                for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                {
                    const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                    const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                    if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                    {
                        existingPtpFactor->markInvalid();
                        factorsToRemove_.push_back(gtsam::Key{factorKey});
                        numFactorsRemoved++;
                        break;
                    }
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
                const gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, adaptiveSigma);
                auto robustNoise = gtsam::noiseModel::Robust::Create(kernel_, noiseModel);
                const auto newFactor = boost::make_shared<PointToPlaneFactor>(
                    keys,
                    states.getImuToLidarExtrinsic(),
                    lidar_points,
                    clusterNormal,
                    clusterCenter,
                    robustNoise,
                    clusterId);
                newSmootherFactors_.add(newFactor);
                clusterFactors_[clusterId] = newFactor;
                numFactorsUpdated++;
                clusterStates_[clusterId] = ClusterState::Idle; // after shifting, cluster goes back to idle state
            }
            break;
            case ClusterState::Pruned: // factor must be removed from the smoother
            {
                const gtsam::NonlinearFactorGraph &smootherFactors = currentSmootherFactors;
                for (size_t factorKey = 0; factorKey < smootherFactors.size(); ++factorKey)
                {
                    const gtsam::NonlinearFactor::shared_ptr existingFactor = smootherFactors[factorKey];
                    const auto existingPtpFactor = boost::dynamic_pointer_cast<PointToPlaneFactor>(existingFactor);
                    if (existingPtpFactor && existingPtpFactor->clusterId_ == clusterId)
                    {
                        existingPtpFactor->markInvalid(); // mark factor as invalid to avoid further optimization
                        // mark existing factor for removal
                        factorsToRemove_.push_back(gtsam::Key{factorKey});
                        numFactorsRemoved++;
                        break;
                    }
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

} // namespace mapping