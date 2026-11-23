/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_INSTRUMENTATION_HPP_
#define MAPPING_BENCHMARKING_INSTRUMENTATION_HPP_

#include <mapping/States.hpp>
#include <mapping/backend/FeatureManager.hpp>
#include <mapping/backend/Smoother.hpp>
#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <gtsam/geometry/Pose3.h>

#include <cstdint>

namespace mapping
{
    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Free functions that fill the `KeyframeTelemetry` columns. Const access to the system only.
        /// @details Called by `KeyframeProbe`, never from the SLAM components.
        namespace instrumentation
        {
            /// @brief Read the values that the update overwrites, right after the keyframe is created.
            /// @details Sets `kf`, `t`, `trigger_*`, `pts_submap` and `voxel_size`.
            /// @param imuPropagated Set to the IMU-propagated pose of the new keyframe.
            void snapshot(KeyframeTelemetry &row, gtsam::Pose3 &imuPropagated, const States &states,
                          uint32_t idxKeyframe);

            /// @brief Fill the post-update columns of `row` (`row.kf` must be set).
            /// @details Each column group (IMU, frontend, geometry, backend) is collected on its own.
            /// When a group throws, it is logged and the other groups are still collected. The columns that the
            /// group set before the throw keep their values, the rest of the group stays `nan`.
            /// IMU and backend columns stay `nan` when `row.lost_tracking` is set.
            ///
            /// Per-factor error:
            /// - `CombinedImuFactor`, `LinearContainerFactor`: `factor->error(estimate)`
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
