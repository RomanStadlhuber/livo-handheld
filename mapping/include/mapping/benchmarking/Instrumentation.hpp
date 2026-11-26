/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_INSTRUMENTATION_HPP_
#define MAPPING_BENCHMARKING_INSTRUMENTATION_HPP_

#include <mapping/States.hpp>
#include <mapping/backend/FeatureManager.hpp>
#include <mapping/backend/Smoother.hpp>
#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstdint>
#include <map>
#include <vector>

namespace mapping
{
    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Free functions that fill the `KeyframeTelemetry` columns. Const access to the system only.
        /// @details Called by `KeyframeProbe`, never from the SLAM components.
        namespace instrumentation
        {
            /// @brief Rows of one `PointToPlaneFactor` on its pose keys, for the `rebuild_*` columns.
            struct ClusterRows
            {
                /// @brief Unwhitened Jacobian block `J_j` (1 x 6) per pose key `X(j)`.
                std::map<gtsam::Key, gtsam::Matrix> jacobians;
                /// @brief Sigma of the noise model, without the robust kernel.
                double sigma = field::NaN;
                /// @brief Robust kernel weight `w` at the residual of `linearize`.
                double weight = field::NaN;
            };
            /// @brief `ClusterRows` per cluster id.
            using ClusterRowsMap = std::map<uint32_t, ClusterRows>;

            /// @brief Read the values that the update overwrites, right after the keyframe is created.
            /// @details Sets `kf`, `t`, `trigger_*`, `pts_submap` and `lidar_fac_before`.
            /// @param imuPropagated Set to the IMU-propagated pose of the new keyframe.
            void snapshot(KeyframeTelemetry &row, gtsam::Pose3 &imuPropagated, const States &states,
                          const Smoother &smoother, uint32_t idxKeyframe);

            /// @brief A `PointToPlaneFactor` in the smoother that touches a marginalized keyframe.
            struct MargFactor
            {
                /// @brief Index of the factor in the smoother graph before the update.
                gtsam::FactorIndex index;
                /// @brief Cluster of the factor.
                uint32_t clusterId;
            };

            /// @brief What `collectMarginalization` needs from the state before the smoother update.
            /// @details Filled by `snapshotMarginalization` in the `KeyframeProbe` constructor, because the update
            /// removes the factors and overwrites the estimate.
            struct MarginalizationSnapshot
            {
                /// @brief Keyframes that the smoother marginalizes in this update, ascending.
                std::vector<uint32_t> keyframes;
                /// @brief Rows of every `PointToPlaneFactor` on a marginalized keyframe, before the rebuild.
                ClusterRowsMap rebuildBaseline;
                /// @brief Every `PointToPlaneFactor` on a marginalized keyframe.
                std::vector<MargFactor> factors;
                /// @brief Smoother estimate before the update, plus the IMU-propagated pose of the new keyframe.
                /// @details The rebuilt factors can hold the point of the new keyframe, which is not in the estimate
                /// yet. All rows of the comparison use these values.
                gtsam::Values values;
            };

            /// @brief Record the marginalization of the update of `idxKeyframe`. Sets `marg_kf`.
            /// @details Call it before the update. A keyframe `j` is marginalized when `X(j)` is in the smoother
            /// estimate and `j + slidingWindowSize < idxKeyframe`. Does nothing else when there is none.
            void snapshotMarginalization(KeyframeTelemetry &row, MarginalizationSnapshot &snapshot,
                                         const States &states, const Smoother &smoother, uint32_t idxKeyframe,
                                         uint32_t slidingWindowSize);

            /// @brief Call after the smoother update. Sets `lidar_fac_marg_*`, `rebuild_*`.
            /// @details Reads the factors that the update added and removed from the `Smoother`.
            /// Do not call it for a `row.lost_tracking` keyframe, it has no update.
            void collectMarginalization(KeyframeTelemetry &row, const MarginalizationSnapshot &snapshot,
                                        const FeatureManager &featureManager, const Smoother &smoother);

            /// @brief Fill the post-update columns of `row` (`row.kf` must be set).
            /// @details Each column group (IMU, frontend, geometry, backend) is collected on its own.
            /// When a group throws, it is logged and the other groups are still collected. The columns that the
            /// group set before the throw keep their values, the rest of the group stays `nan`.
            /// IMU and backend columns stay `nan` when `row.lost_tracking` is set.
            ///
            /// Per-factor error:
            /// - `CombinedImuFactor`: `factor->error(estimate)`
            /// - `PointToPlaneFactor`: `evaluateErrorDiagnostic(estimate)`, the unweighted error (`err_lidar`)
            ///   and the IRLS weight of the robust kernel (`w_lidar`) at the same residual that `linearize` uses.
            ///   Its `unwhitenedError` throws by design, so `factor->error()` is not available.
            ///
            /// Information matrix for `eig_*_min`: sum over `Tracked` clusters with a point in this keyframe of
            /// `J^T J / s`, `J = [(p x n)^T, n^T]`, `p` the point and `n` the cluster normal in the keyframe IMU frame,
            /// `s = FeatureManager::factorVariance`. Rotation block first, as in `gtsam::Pose3`.
            void collect(KeyframeTelemetry &row, const gtsam::Pose3 &imuPropagated, const States &states,
                         const FeatureManager &featureManager, const Smoother &smoother) noexcept;
        } // namespace instrumentation
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_INSTRUMENTATION_HPP_
