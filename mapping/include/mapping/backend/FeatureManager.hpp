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
#include <mapping/Config.hpp>
#include <mapping/factors/PointToPlaneFactor.hpp>

#include <unordered_map> // cluster->factorIdx mapping with O(1) access
#include <iostream>
#include <utility>
#include <list>
#include <set>
#include <vector>

namespace mapping
{
    /// @ingroup backend_features
    /// @brief A class to manage the feature clusters,
    /// their statesand associated GTSAM factors.
    class FeatureManager
    {
    public:
        FeatureManager();
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
        /// Will internally call `removePointFromCluster` for each cluster that contains a point from the marginalized
        /// keyframe,
        ///
        /// NOTE: will also create a set of factors to use for marginalization.
        /// @param idxKeyframe Index of the keyframe to be marginalized.
        /// @param markovBlanket Smoother estimate of the poses `X(idxKeyframe..)` and calibration variables.
        /// @param currentSmootherFactors The current factor graph of the smoother.
        /// With `anchorMarginalization_` the point of the keyframe stays in the cluster as a constant world point
        /// ("anchor", `clusterAnchors_`) and no unary factor is created.
        /// @param states Used to read the world position of the marginalized point (anchor).
        void removeKeyframeFromClusters(const States &states, const uint32_t &idxKeyframe,
                                        const gtsam::Values &markovBlanket,
                                        const gtsam::NonlinearFactorGraph &currentSmootherFactors);
        /// @brief Add a new point to a cluster when it is successfully tracked against it.
        void addPointToCluster(const ClusterId &clusterId, const SubmapIdxPointIdx &pointIdx,
                               const double &planeThickness);
        /// @brief Remove a point when 6-sigma test fails.
        /// Note that after this, "updateClusterParameters" still needs to be called explicitly (with
        /// recalcPlaneThickness=true).
        /// @param firstInHistory Set this to true if you are marginalizing the oldest keyframe from the cluster.
        void removePointFromCluster(const ClusterId &clusterId, const uint32_t &idxKeyframe,
                                    bool firstInHistory = false);
        /// @brief Update cluster parameters from scratch
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States &states, const ClusterId &clusterId, bool recalcPlaneThickness,
                                     const MappingConfig &config);
        /// @brief Set new cluster parameters including thickness (i.e. valid tracking and point was added)
        /// @details **Important:** assumes that the plane thickness history was updated accordingly beforehand!
        void updateClusterParameters(const States &states, const ClusterId &clusterId, const Eigen::Vector3d &newNormal,
                                     const Eigen::Vector3d &newCenter);
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
        createAndUpdateFactors(const States &states, const gtsam::NonlinearFactorGraph &currentSmootherFactors);

        /// @brief Clear all clusters and factor bookkeeping.
        void reset();

        /// @brief Shorthand check for whether a cluster is valid (= tracking or idle).
        bool isClusterValid(const ClusterId &clusterId) const
        {
            return !(clusterStates_.at(clusterId) == ClusterState::Premature ||
                     clusterStates_.at(clusterId) == ClusterState::Pruned);
        };
        /// @brief Summarize current clusters for debugging purposes.
        void summarizeClusters() const;
        /// @brief Summarize current factors for debugging purposes.
        /// @param factors The factor graph to summarize obtained from `Smoother::getFactors()`.
        void summarizeFactors(const gtsam::NonlinearFactorGraph &factors) const;

    private:
        /// @brief Compute the twists required for extrapolation based on temporal calibration.
        /// @param states The current states used to obtain the IMU frame body poses and timestamps.
        /// @return Mapping of `gtsam::Key` to `(angVel, linVel)` twists between all states in the sliding window,
        /// `k-w, k-w+1, ..., k`, where `twist[k]` is used to extrapolate the pose of `k`.
        std::map<gtsam::Key, std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        computeTemporalCalibrationTwists(const States &states);

    public: // attributes
        /**
         * NOTE: feature attributes get updated a lot, so currently it makes sense to make them public
         */

        std::map<ClusterId, std::map<uint32_t, std::size_t>> clusters_;
        /// @brief Constant world points of marginalized keyframes per cluster ("anchors"), only filled when
        /// `anchorMarginalization_` is set. They enter the plane fit and the factor, but not the KNN query,
        /// the gating test or the thickness history.
        std::map<ClusterId, std::vector<Eigen::Vector3d>> clusterAnchors_;
        /// @brief Keyframe that created each anchor, same order and size as `clusterAnchors_`.
        std::map<ClusterId, std::vector<uint32_t>> clusterAnchorKeyframes_;
        /// @brief Experimental: keep marginalized points as anchors instead of a unary factor
        /// (`backend.marginalization_anchor`).
        bool anchorMarginalization_{false};
        /// @brief Maximum number of anchors per cluster (`lidar_frontend.clustering.max_points`), the oldest anchor is
        /// dropped first. 0: no cap.
        std::size_t maxAnchors_{0};
        /// @brief The thickness of each added cluster track, mean squared point-to-plane distance [m^2]
        std::map<ClusterId, std::vector<double>> clusterPlaneThicknessHistory_;
        /// @brief Mean of the squared thickness history [m^4], used for validation and modelling noise characteristics.
        std::map<ClusterId, double> clusterPlaneThickness_;
        /// @brief `clusterPlaneThickness_` plus a `1e-4` floor [m^4], used for the front-end gating test only.
        std::map<ClusterId, double> clusterSigmas_;
        std::map<ClusterId, ClusterState> clusterStates_;
        /// @brief Cached cluster centroids and normals for fast access during tracking and formulating smoothing
        /// constraints.
        std::map<ClusterId, std::shared_ptr<Eigen::Vector3d>> clusterCenters_, clusterNormals_;
        /// @brief Counter for assigning unique IDs to clusters.
        ClusterId clusterIdCounter_{0};
        // factor management for what's passed to the smoother
        gtsam::NonlinearFactorGraph newSmootherFactors_;
        gtsam::FactorIndices factorsToRemove_;
        /// @brief Robust kernel for point-to-plane factors, built from the config.
        /// see also: https://gtsam.org/doxygen/a03860.html
        /// The threshold is in whitened units, as the residual is whitened before the kernel is applied.
        gtsam::noiseModel::mEstimator::Base::shared_ptr kernel_{gtsam::noiseModel::mEstimator::Huber::Create(1.345)};

    public:
        /// @brief Build the robust kernel of the LiDAR factors from the config.
        void setLidarKernel(const LidarKernelConfig &config);
        /// @brief Anchors of a cluster, empty if there are none.
        const std::vector<Eigen::Vector3d> &anchorsOf(const ClusterId &clusterId) const
        {
            static const std::vector<Eigen::Vector3d> none;
            const auto it = clusterAnchors_.find(clusterId);
            return it == clusterAnchors_.end() ? none : it->second;
        }
        /// @brief Set optional calibration keys for temporal and extrinsic calibration.
        void setCalibrationKeys(boost::optional<gtsam::Key> dtKey, boost::optional<gtsam::Key> extrinsicKey);
        // Sensor + pose noise floor added to plane-fit variance.
        // Plane thickness alone only captures fit quality, not LiDAR ranging noise
        // or residual pose uncertainty at evaluation time.
        static constexpr double SENSOR_VARIANCE = 1.6e-7; // [m^4] (variance!)
        /// @brief Variance [m^4] of the point-to-plane factor noise model of a cluster: `mean(thickness^2)` plus
        /// `SENSOR_VARIANCE`, without the `1e-4` floor (see BA-LINS eq. 14, MSC-LIO).
        /// @note Differs from `clusterSigmas_`, which keeps the floor for the front-end gating test.
        double factorVariance(const ClusterId clusterId) const
        {
            return clusterPlaneThickness_.at(clusterId) + SENSOR_VARIANCE;
        }
        /// @brief Add or update the factor index mapping for a given cluster id.
        /// @param clusterId Id of the cluster.
        /// @param factorIdx Index of the factor in the `NonlinearFactorGraph` as returned by
        /// `gtsam::IncrementalFixedLagSmoother::getISAM2Result().newFactorsIndices`.
        void setClusterFactorIdxMapping(const ClusterId clusterId, gtsam::FactorIndex factorIdx)
        {
            clusterFactorIdxMap_[clusterId] = factorIdx;
        };

        bool hasClusterFactorIdx(const ClusterId clusterId)
        {
            return clusterFactorIdxMap_.find(clusterId) != clusterFactorIdxMap_.end();
        };

#ifdef MAPPING_BENCHMARKING
        /// @brief Benchmark build only: what the feature manager did to the LiDAR factors for one keyframe.
        struct KeyframeFactorStats
        {
            /// @brief `removeKeyframeFromClusters`: number of marginalization factors, summed whitened information
            /// `sum(H^T H)` of their `X(k)` blocks (rotation first), and the kernel weight of each source factor.
            /// Sums over all keyframes that are marginalized before the same `createAndUpdateFactors` call.
            std::size_t numMargFactors = 0;
            Eigen::Matrix<double, 6, 6> margInformation = Eigen::Matrix<double, 6, 6>::Zero();
            std::vector<double> margWeights;
            /// @brief `removeKeyframeFromClusters`: clusters set to `Pruned` / `ShiftedIdle`.
            std::size_t numPruned = 0, numShiftedIdle = 0;
            /// @brief `createAndUpdateFactors`: fresh factors, re-added factors (old factor removed and rebuilt),
            /// and factor indices passed to the smoother for removal.
            std::size_t numFactorsAdded = 0, numFactorsUpdated = 0, numFactorsRemoved = 0;
            /// @brief `releaseClustersInsideBlock`: clusters whose factor stays in the smoother and goes whole into
            /// the iSAM2 marginalization (block release).
            std::set<ClusterId> wholeReleased;
        };
        /// @brief Benchmark build only: stats of the last `createAndUpdateFactors` call, together with the stats of all
        /// `removeKeyframeFromClusters` calls before it.
        const KeyframeFactorStats &lastKeyframeFactorStats() const { return lastKeyframeFactorStats_; }

        /// @brief Benchmark build only (block release): hand the clusters that lie inside a released block whole to
        /// the iSAM2 marginalization.
        /// @details A cluster goes whole when it has a live factor, and all pose keys of the factor and all points of
        /// the cluster are in `block` or in `idxNext` (the IMU neighbor `X(j+1)`, fixed anyway). Its factor stays in
        /// the smoother and the cluster is erased here, so `createAndUpdateFactors` does not touch the factor and no
        /// unary factor is created. Call it before `removeKeyframeFromClusters` for the keyframes of the block, which
        /// then sheds the crossing clusters as before.
        /// @param block The released keyframes, contiguous.
        /// @param idxNext The keyframe after the newest released keyframe.
        /// @return Number of clusters released whole.
        std::size_t releaseClustersInsideBlock(const std::vector<uint32_t> &block, uint32_t idxNext,
                                               const gtsam::NonlinearFactorGraph &currentSmootherFactors);

    private:
        KeyframeFactorStats lastKeyframeFactorStats_;
        /// @brief Benchmark build only: accumulates the stats until `createAndUpdateFactors` moves them to
        /// `lastKeyframeFactorStats_`.
        KeyframeFactorStats pendingKeyframeFactorStats_;
#endif

    private:
        /// @brief Erase a cluster and all its bookkeeping (not its factor in the smoother).
        void eraseCluster(const ClusterId &clusterId);

        boost::optional<gtsam::Key> dtKey_;
        boost::optional<gtsam::Key> extrinsicKey_;
        /// @brief Mapping from Cluster ID to associated factor in the graph
        std::unordered_map<ClusterId, gtsam::FactorIndex> clusterFactorIdxMap_;
    };
} // namespace mapping

#endif // MAPPING_BACKEND_FEATUREMANAGER_HPP_
