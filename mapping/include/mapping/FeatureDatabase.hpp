#pragma once

#ifndef MAPPING_FEATUREDATABASE_HPP_
#define MAPPING_FEATUREDATABASE_HPP_

#include <mapping/types.hpp>
#include <mapping/States.hpp>
#include <list>

namespace mapping
{
    /// @brief A class to manage the feature clusters and their states.
    class FeatureDatabase
    {
    public:
        FeatureDatabase() = default;
        ~FeatureDatabase() = default;

        /// @brief Find all clusters that are associated with a given keyframe.
        /// Could be useful for marginalization.
        /// @param idxKeyframe The index of the keyframe to find associated clusters for.
        /// @return A list of cluster IDs that are associated with the given keyframe.
        std::list<ClusterId> findClustersAssociatedWithKeyframe(const uint32_t &idxKeyframe) const;

        /// @brief Remove a keyframe association from all clusters, and update cluster states accordingly.
        /// Will internally call `removePointFromCluster` for each cluster that contains a point from the marginalized keyframe,
        void removeKeyframeFromClusters(const uint32_t &idxKeyframe);

        /// @brief Add a new point to a cluster when it is successfully tracked against it.
        void addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness);
        /// @brief Remove a point when 6-sigma test fails.
        /// Note that after this, "updateClusterParameters" still needs to be called explicitly (with recalcPlaneThickness=true).
        /// @param firstInHistory Set this to true if you are marginalizing the oldest keyframe from the cluster.
        void removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory = false);

        /// @brief Update cluster parameters from scratch
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States& states, const ClusterId &clusterId, bool recalcPlaneThickness);
        /// @brief Set new cluster parameters including thickness (i.e. valid tracking and point was added)
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States& states, const ClusterId &clusterId, const Eigen::Vector3d &newNormal, const Eigen::Vector3d &newCenter);
    
    public: // attributes
        /**
         * NOTE: feature attributes get updated a lot, so currently it makes sense to make them public
         */

        std::map<ClusterId, std::map<uint32_t, std::size_t>> clusters_;
        /// @brief The thickness of each added cluster track
        std::map<ClusterId, std::vector<double>> clusterPlaneThicknessHistory_;
        /// @brief The thickness of a clusters fitted plane, used for validation and modelling noise characteristics.
        std::map<ClusterId, double> clusterPlaneThickness_;
        std::map<ClusterId, double> clusterSigmas_;
        std::map<ClusterId, ClusterState> clusterStates_;
        /// @brief Cached cluster centroids and normals for fast access during tracking and formulating smoothing constraints.
        std::map<ClusterId, std::shared_ptr<Eigen::Vector3d>> clusterCenters_, clusterNormals_;
    };
} // namespace mapping

#endif // MAPPING_FEATUREDATABASE_HPP_