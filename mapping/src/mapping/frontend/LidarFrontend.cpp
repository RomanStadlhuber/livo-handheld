/// @file
/// @ingroup frontend_lidar
#include <mapping/frontend/LidarFrontend.hpp>
#include <mapping/helpers.hpp> // planeFitSVD

namespace mapping
{
    std::shared_ptr<open3d::geometry::PointCloud> LidarFrontend::accumulateUndistortedScans(
        const States &states, Buffers &buffers, const MappingConfig &config)
    {
        // Merge buffered scans together & create new keyframe submap
        open3d::geometry::PointCloud newSubmap;
        /**
         * NOTE: the scan buffer contains the undistorted scan pointclouds in their own scan origin frame,
         * along with the pose that the scan has w.r.t. the last keyframe.
         * In order to build the new submap at the new keyframe origin, all buffered scans need to be transformed
         * by their pose w.r.t. the new keyframe first.
         *
         * This means inverting the pose of the last scan to the last keyframe, then composing it with each scan's
         * pose to the last keyframe to get the pose of each scan w.r.t. the new keyframe.
         *
         * It is important that the scans in the buffer are not transformed before this.
         */
        const std::list<ScanBuffer> &scanBuffer = buffers.getScanBuffer();
        const gtsam::Pose3 newKf_T_lastKf = scanBuffer.rbegin()->kf_T_scan->inverse();
        for (auto reverseIt = scanBuffer.rbegin(); reverseIt != scanBuffer.rend(); ++reverseIt)
        {
            const ScanBuffer &scan = *reverseIt;
            // pose of the scan w.r.t. the new keyframe
            const gtsam::Pose3 newKf_T_scan = newKf_T_lastKf.compose(*(scan.kf_T_scan));
            // move undistorted pcd to the origin of the new keyframe
            scan.pcd->Transform(newKf_T_scan.matrix());
            newSubmap += *(scan.pcd);
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(config.lidar_frontend.voxel_size);
        return ptrNewSubmapVoxelized;
    }

    bool LidarFrontend::trackScanPointsToClusters(
        const uint32_t &idxKeyframe,
        const States &states,
        FeatureManager &featureManager,
        const MappingConfig &config)
    {
        // Search for same-plane point clusters among all keyframes
        std::vector<int> knnIndices(config.lidar_frontend.knn_neighbors);
        knnIndices.reserve(config.lidar_frontend.knn_neighbors);
        std::vector<double> knnDists(config.lidar_frontend.knn_neighbors);
        knnDists.reserve(config.lidar_frontend.knn_neighbors);
        std::size_t validTracks = 0, numValidClusters = 0;

#ifdef ENABLE_DBG_CMP
        // per-keyframe counters for debugging cluster-tracking behavior
        struct TrackingStatistics
        {
            std::size_t enteredTotal{0};
            std::size_t enteredPremature{0};
            std::size_t enteredIdle{0};
            std::size_t enteredShiftedIdle{0};
            std::size_t enteredTracked{0};
            std::size_t rejKnnInsufficient{0};
            std::size_t rejPlaneFitInvalid{0};
            std::size_t rejGating{0};
            std::size_t rejConsistency{0};
            std::size_t promotedTracked{0};
        } stats;
#endif
        std::map<uint32_t, std::shared_ptr<open3d::geometry::PointCloud>> &keyframeSubmaps = states.getKeyframeSubmaps();
        // KD-Tree of the current submap, used for cluster tracking
        const open3d::geometry::KDTreeFlann kdTree{*keyframeSubmaps[idxKeyframe]};
        // project each cluster point onto the current keyframe and try to find the 5 nearest neighbors
        for (auto const &cluster : featureManager.clusters_)
        {
            auto const &[clusterId, clusterPointIdxs] = cluster;
            const ClusterState clusterState = featureManager.clusterStates_.at(clusterId);
            // --- ignore pruned clusters ---
            if (clusterState == ClusterState::Pruned)
                continue;
#ifdef ENABLE_DBG_CMP
            stats.enteredTotal++;
            switch (clusterState)
            {
            case ClusterState::Premature:   stats.enteredPremature++;   break;
            case ClusterState::Idle:        stats.enteredIdle++;        break;
            case ClusterState::ShiftedIdle: stats.enteredShiftedIdle++; break;
            case ClusterState::Tracked:     stats.enteredTracked++;     break;
            default: break;
            }
#endif
            // --- tracking: KNN search & SVD plane fit ---
            auto const &[idxClusterKF, idxSubmapPt] = *clusterPointIdxs.rbegin(); // use newest cluster pt as KNN query
            const Eigen::Vector3d &world_clusterPt = keyframeSubmaps[idxClusterKF]->points_[idxSubmapPt];
            const int knnFound = kdTree.SearchHybrid(
                world_clusterPt,
                config.lidar_frontend.knn_radius,
                config.lidar_frontend.knn_neighbors,
                knnIndices,
                knnDists);
            // skip if not enough points within radius; still update cluster parameters from new poses
            if(knnFound < config.lidar_frontend.knn_neighbors)
            {
#ifdef ENABLE_DBG_CMP
                stats.rejKnnInsufficient++;
#endif
                if (clusterState == ClusterState::Premature) // don't update premature clusters
                    continue;
                featureManager.clusterStates_[clusterId] = ClusterState::Idle;
                featureManager.updateClusterParameters(states, clusterId, false, config); // update cluster, keep thickness (no new KF association)
                continue;
            }
            // collect KNN points and fit a plane
            std::vector<Eigen::Vector3d> knnPoints;
            knnPoints.reserve(knnFound);
            for (int i = 0; i < knnFound; ++i)
                knnPoints.push_back(keyframeSubmaps[idxKeyframe]->points_[knnIndices[i]]);
            const auto [validPlaneTrack, planeTrackNormal, knnCenter, knnPointsMat, planeTrackThickness] = planeFitSVD(
                knnPoints,
                config.lidar_frontend.planarity_check.planarity,
                config.lidar_frontend.planarity_check.linearity);
            // --- tracking failed (KNN plane fit invalid): update cluster center & normal, keep thickness ---
            if (!validPlaneTrack || planeTrackThickness > config.lidar_frontend.clustering.max_plane_thickness)
            {
#ifdef ENABLE_DBG_CMP
                stats.rejPlaneFitInvalid++;
#endif
                if (clusterState == ClusterState::Premature) // don't update premature clusters
                    continue;
                featureManager.clusterStates_[clusterId] = ClusterState::Idle;
                featureManager.updateClusterParameters(states, clusterId, false, config); // update cluster, keep thickness (no new KF association)
                continue;
            }
            static constexpr size_t IDX_KNN_POINT = 1; // use second-nearest neighbor for tracking to increase plane stability
            // -- 6-sigma test for new plane point with current cluster center & plane normal ---
            if (clusterState != ClusterState::Premature)
            {
                const Eigen::Vector3d &knnPoint = keyframeSubmaps[idxKeyframe]->points_[knnIndices[IDX_KNN_POINT]];
                const double pointToPlaneDist = std::abs(
                    featureManager.clusterNormals_.at(clusterId)->dot(knnPoint - *featureManager.clusterCenters_.at(clusterId)));
                // NOTE: see paper MSC-LIO, Eq. 19 -> they use this adaptive sigma formulation for the gating test.
                const double adaptiveSigma = std::pow(featureManager.clusterSigmas_.at(clusterId), 0.25);
                if (pointToPlaneDist >= 3.0 * adaptiveSigma)
                {
#ifdef ENABLE_DBG_CMP
                    stats.rejGating++;
#endif
                    featureManager.clusterStates_[clusterId] = ClusterState::Idle;
                    featureManager.updateClusterParameters(states, clusterId, false, config); // update cluster, keep thickness (no new KF association)
                    continue;
                }
            }
            // --- tracking valid: add point to cluster ---
            // associate the second-nearest point with the cluster to increase stability
            featureManager.addPointToCluster(clusterId, {idxKeyframe, knnIndices[IDX_KNN_POINT]}, planeTrackThickness);
            validTracks++;
            knnIndices.clear();
            knnDists.clear();
            const bool clusterTooSmall = clusterPointIdxs.size() < config.lidar_frontend.clustering.min_points;
            if (clusterTooSmall || clusterState == ClusterState::Pruned)
                continue; // skip
            // collect all points associated with this cluster
            std::vector<Eigen::Vector3d> clusterPoints;
            clusterPoints.reserve(clusterPointIdxs.size());
            for (auto const &pointIdxPair : clusterPointIdxs)
            {
                const auto &[idxSubmap, idxPoint] = pointIdxPair;
                clusterPoints.push_back(keyframeSubmaps[idxSubmap]->points_[idxPoint]);
            }
            const auto [planeValid, planeNormal, clusterCenter, clusterPointsMat, planeThickness] = planeFitSVD(
                clusterPoints,
                config.lidar_frontend.planarity_check.planarity,
                config.lidar_frontend.planarity_check.linearity);
            // --- new plane normal consistency check ---
            if (
                clusterState == ClusterState::Premature // for premature clusters, update immediately
                // otherwise perform consistency check
                || (planeValid && std::abs(featureManager.clusterNormals_.at(clusterId)->dot(planeNormal)) > config.lidar_frontend.clustering.normal_consistency_threshold))
            {
                featureManager.clusterStates_[clusterId] = ClusterState::Tracked;
                // Note: internally uses thickness history to update covariance
                featureManager.updateClusterParameters(states, clusterId, planeNormal, clusterCenter);
                numValidClusters++;
#ifdef ENABLE_DBG_CMP
                stats.promotedTracked++;
#endif
            }
            else
            {
#ifdef ENABLE_DBG_CMP
                stats.rejConsistency++;
#endif
                // conistency check failed -> mark cluster Idle, remove added pt and recompute parameters from old old associations
                if (clusterState == ClusterState::Premature) // must not go from premature to idle
                    continue;
                featureManager.clusterStates_[clusterId] = ClusterState::Idle;    // idle - no valid track in newest KF
                featureManager.removePointFromCluster(clusterId, idxKeyframe);    // remove latest association
                featureManager.updateClusterParameters(states, clusterId, false, config); // update location, thickness shouldn't change (no new KF association)
                continue;
            }
        }
        if (validTracks == 0) // abort if the latest frame could not be tracked
        {
            return false;
        }
#ifdef ENABLE_DBG_CMP
        std::cout << "::: [INFO] keyframe " << idxKeyframe << " had " << validTracks << " tracks and " << numValidClusters << " valid clusters :::" << std::endl;
        std::cout << "::: [DEBUG] keyframe " << idxKeyframe << " tracking stats :::\n"
                  << "\tentered: " << stats.enteredTotal
                  << " (premature: " << stats.enteredPremature
                  << ", idle: " << stats.enteredIdle
                  << ", shifted: " << stats.enteredShiftedIdle
                  << ", tracked: " << stats.enteredTracked << ")\n"
                  << "\trejected: knn=" << stats.rejKnnInsufficient
                  << " planeFit=" << stats.rejPlaneFitInvalid
                  << " gating=" << stats.rejGating
                  << " consistency=" << stats.rejConsistency << "\n"
                  << "\tpromoted to Tracked: " << stats.promotedTracked
                  << ", valid tracks: " << validTracks << std::endl;
#endif
        return idxKeyframe < config.lidar_frontend.clustering.min_points ? true : numValidClusters > 0;
    }
}
