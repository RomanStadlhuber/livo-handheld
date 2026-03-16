/// @file
/// @ingroup backend_features
#pragma once

#ifndef MAPPING_BACKEND_FEATUREMANAGER_HPP_
#define MAPPING_BACKEND_FEATUREMANAGER_HPP_

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/CombinedImuFactor.h> // for "summarizeFactors"

#include <mapping/types.hpp>
#include <mapping/States.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>

#include <iostream>
#include <utility>
#include <list>
#include <set>

namespace mapping
{
    /// @ingroup backend_features
    /// @brief A class to manage the feature clusters,
    /// their statesand associated GTSAM factors.
    class FeatureManager
    {
    public:
        FeatureManager() = default;
        ~FeatureManager() = default;

        /// @brief Create new clusters from points in a keyframe.
        /// @param states The current system states.
        /// @param idxKeyframe The index of the keyframe to create clusters from.
        /// @param voxelSize The voxel size to use for downsampling. Uses all points when `<= 0.01`.
        void createNewClusters(const States &states, const uint32_t &idxKeyframe, double voxelSize);
        /// @brief Remove clusters associated with a keyframe, including their parameters.
        /// @details Pruning should only start when the sliding window size is first exceeded.
        void pruneClusters(const uint32_t &idxKeyframe);
        /// @brief Find all clusters that are associated with a given keyframe.
        /// Could be useful for marginalization.
        /// @param idxKeyframe The index of the keyframe to find associated clusters for.
        /// @return A list of cluster IDs that are associated with the given keyframe.
        std::list<ClusterId> findClustersAssociatedWithKeyframe(const uint32_t &idxKeyframe) const;
        /// @brief Remove a keyframe association from all clusters, and update cluster states accordingly.
        /// Will internally call `removePointFromCluster` for each cluster that contains a point from the marginalized keyframe,
        ///
        /// NOTE: will also create a set of factors to use for marginalization.
        /// @param idxKeyframe Index of the keyframe to be marginalized.
        void removeKeyframeFromClusters(const uint32_t &idxKeyframe, const gtsam::Values &markovBlanket);
        /// @brief Add a new point to a cluster when it is successfully tracked against it.
        void addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx, const double &planeThickness);
        /// @brief Remove a point when 6-sigma test fails.
        /// Note that after this, "updateClusterParameters" still needs to be called explicitly (with recalcPlaneThickness=true).
        /// @param firstInHistory Set this to true if you are marginalizing the oldest keyframe from the cluster.
        void removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe, bool firstInHistory = false);
        /// @brief Update cluster parameters from scratch
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States &states, const ClusterId &clusterId, bool recalcPlaneThickness);
        /// @brief Set new cluster parameters including thickness (i.e. valid tracking and point was added)
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States &states, const ClusterId &clusterId, const Eigen::Vector3d &newNormal, const Eigen::Vector3d &newCenter);
        /// @brief Create new factors for previously unttracked clusters and update existing factors.
        /// **Warning:** re-keying (i.e. modifying factor-key associations in-place) is not tracked by
        /// GTSAM, i.e. the factor graph is treated as static after initialization for the solver.
        /// While iSAM2 does keep state (bayes-tree) to track which variables need to be relinearized,
        /// it does not re-key internally ([see also](https://github.com/borglab/gtsam/issues/149)).
        /// This means that updates to factor-key associations can only be passed through copy-add-remove.
        /// @details The return value of this function must be passed to the smoother and cannot be discarded.
        /// @param states The current system states, used for accessing the latest estimates and timestamps.
        /// @param currentSmootherFactors The current factor graph of the smoother obtain with `smoother.getFactors()`.
        /// @return New factors to add and list of factors to remove with `smoother.update(...)`
        [[nodiscard]] std::pair<gtsam::NonlinearFactorGraph, gtsam::FactorIndices>
        createAndUpdateFactors(
            const States &states,
            const gtsam::NonlinearFactorGraph &currentSmootherFactors);

        /// @brief Clear all clusters and factor bookkeeping.
        void reset();

        /// @brief Shorthand check for whether a cluster is valid (= tracking or idle).
        bool isClusterValid(const ClusterId &clusterId) const
        {
            return !(clusterStates_.at(clusterId) == ClusterState::Premature || clusterStates_.at(clusterId) == ClusterState::Pruned);
        };
        /// @brief Summarize current clusters for debugging purposes.
        void summarizeClusters() const;
        /// @brief Summarize current factors for debugging purposes.
        /// @param factors The factor graph to summarize obtained from `Smoother::getFactors()`.
        void summarizeFactors(const gtsam::NonlinearFactorGraph &factors) const;

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
        std::map<ClusterId, boost::shared_ptr<PointToPlaneFactor>> clusterFactors_;
        /// @brief Counter for assigning unique IDs to clusters.
        ClusterId clusterIdCounter_{0};
        // factor management for what's passed to the smoother
        gtsam::NonlinearFactorGraph newSmootherFactors_;
        gtsam::FactorIndices factorsToRemove_;
        /// @brief Robust kernel for point-to-plane factors
        /// see also: https://gtsam.org/doxygen/a03860.html
        gtsam::noiseModel::mEstimator::GemanMcClure::shared_ptr kernel_{
            gtsam::noiseModel::mEstimator::GemanMcClure::shared_ptr(new gtsam::noiseModel::mEstimator::GemanMcClure(1.0))};

    public:
        /// @brief Set optional calibration keys for temporal and extrinsic calibration.
        void setCalibrationKeys(boost::optional<gtsam::Key> dtKey, boost::optional<gtsam::Key> extrinsicKey);

    private:
        boost::optional<gtsam::Key> dtKey_;
        boost::optional<gtsam::Key> extrinsicKey_;
    };
} // namespace mapping

#endif // MAPPING_BACKEND_FEATUREMANAGER_HPP_