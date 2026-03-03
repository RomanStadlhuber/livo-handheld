#include <mapping/FeatureDatabase.hpp>
#include <mapping/helpers.hpp> // planeFitSVD

namespace mapping
{
    void FeatureDatabase::addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness)
    {
        auto const &[idxSubmap, idxPoint] = pointIdx; // destructure to make it clearer what's going on
        clusters_[clusterId][idxSubmap] = idxPoint;
        clusterPlaneThicknessHistory_[clusterId].push_back(planeThickness);
    }

    std::list<ClusterId> FeatureDatabase::findClustersAssociatedWithKeyframe(const uint32_t &idxKeyframe) const
    {
        std::list<ClusterId> associatedClusters;
        for (const auto &[clusterId, keyframeAssociations] : clusters_)
        {
            if (keyframeAssociations.find(idxKeyframe) != keyframeAssociations.end())
                associatedClusters.push_back(clusterId);
        }
        return associatedClusters;
    }

    void FeatureDatabase::removeKeyframeFromClusters(const uint32_t &idxKeyframe)
    {
        std::size_t numMarginalized = 0;
        for (auto const &[clusterId, clusterPoints] : clusters_)
        {
            auto itPoint = clusterPoints.find(idxKeyframe);
            if (itPoint != clusterPoints.end())
            {
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

    void FeatureDatabase::removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory)
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

    void FeatureDatabase::updateClusterParameters(const States &states, const ClusterId &clusterId, bool recalcPlaneThickness)
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

    void FeatureDatabase::updateClusterParameters(
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

} // namespace mapping