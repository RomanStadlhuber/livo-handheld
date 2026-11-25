/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_KEYFRAMETELEMETRY_HPP_
#define MAPPING_BENCHMARKING_KEYFRAMETELEMETRY_HPP_

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

/// @brief Schema of `telemetry.csv`, one row per keyframe that `MappingSystem::track()` creates.
/// @details Each entry is `X(Type, name)`. The list generates the members of `KeyframeTelemetry`, the CSV header
/// and the CSV row writer, so a new column is one line here plus the code that sets its value
/// (see `mapping/benchmarking/Instrumentation.cpp`). Types, see namespace `mapping::benchmarking::field`:
/// - `Index`: unsigned integer, `nan` when not set
/// - `Flag`: `0` / `1`
/// - `Real`: double, `nan` when not set
/// - `Time`: system time `[s]`, written as absolute bag time (`%.9f`), same value as in `trajectory.tum`
/// - `Stats`: distribution of a sample, 6 columns `<name>_n`, `_min`, `_max`, `_median`, `_mean`, `_std`
///
/// Column meaning:
/// - `kf`: keyframe index
/// - `t`: keyframe timestamp
/// - `lost_tracking`: `trackScanPointsToClusters` returned false at this keyframe
///   (IMU and backend columns are `nan` then)
/// - `trigger_dist` `[m]`, `trigger_angle` `[rad]`: previous keyframe IMU pose to the IMU-propagated pose,
///   same values that `track()` compares with the keyframe thresholds
/// - `trigger_dt` `[s]`: time since the previous keyframe
/// - `imu_pred_dpos` `[m]`, `imu_pred_drot` `[rad]`: distance / angle between the IMU-propagated and the optimized
///   pose of the current keyframe (magnitude of smoother correction)
/// - `vel_norm` `[m/s]`, `ba_norm`, `bg_norm`: optimized velocity, accelerometer and gyroscope bias norms
/// - `pts_submap`: points in the keyframe submap after the voxel filter
/// - `clusters_valid`: `numValidClusters` of `LidarFrontend::trackScanPointsToClusters` at the current keyframe,
///   `nan` when it returned false (see `LidarFrontend::keyframeNumValidClusters`)
/// - `thickness`: newest plane thickness history entry of the `Tracked` clusters with a point in the current
///   keyframe. `thickness_n` can be slightly larger than `clusters_valid` (a cluster on the `clusterTooSmall`
///   path also gets a point in the current keyframe but is not counted)
/// - `eig_rot_min`, `eig_trans_min`: smallest eigenvalue of the rotation and translation blocks
///   of the point-to-plane information matrix of this keyframe pose
/// - `err_imu`, `err_lidar`: per-factor error of all `CombinedImuFactor` / `PointToPlaneFactor` in the smoother.
///   `err_lidar` is the unweighted error `0.5 * r^2 / sigma^2` of `PointToPlaneFactor::evaluateErrorDiagnostic`,
///   without the robust kernel.
/// - `w_lidar`: IRLS weight `w(r)` of the robust kernel per `PointToPlaneFactor` (`1`: full weight, `-> 0`:
///   switched off by the kernel). The optimizer solves for `w_lidar * err_lidar` per factor.
///   Pick the "rejected" threshold in the evaluation, it depends on the kernel and its scale.
/// - `lidar_var` `[m^4]`: variance of the noise model (without the robust kernel) per `PointToPlaneFactor`
///   in the smoother
/// - `cov_rot_trace`, `cov_trans_trace`: trace of the rotation and translation blocks of the marginal covariance
///   of `X(kf)`
///
/// LiDAR factor accounting of the smoother update at this keyframe. The counts satisfy
/// `lidar_fac_after = lidar_fac_before - lidar_fac_removed + lidar_fac_readded + lidar_fac_new
///  - lidar_fac_marg_pruned - lidar_fac_marg_lost`.
/// - `lidar_fac_before`, `lidar_fac_after`: `PointToPlaneFactor` in the smoother before the marginalization of this
///   keyframe and after the smoother update
/// - `lidar_fac_removed`: factors that `createAndUpdateFactors` passes to the smoother for removal
/// - `lidar_fac_readded`: factors that `createAndUpdateFactors` rebuilds for a cluster with a removed factor
///   (`Tracked` with a live factor, `ShiftedIdle`)
/// - `lidar_fac_new`: factors that `createAndUpdateFactors` creates for a cluster without a live factor
/// - `lidar_fac_marg_pruned`, `lidar_fac_marg_lost`: factors on a marginalized keyframe that are not removed by
///   `createAndUpdateFactors`, so the iSAM2 marginalization absorbs them with all their keys.
///   `_pruned`: the cluster is `Pruned` (fewer than 3 points left). `_lost`: the cluster is still alive, but has no
///   factor after the update (e.g. `ShiftedIdle` overwritten with `Idle` by a failed track).
/// - `clusters_pruned`, `clusters_shifted_idle`: clusters that `removeKeyframeFromClusters` sets to `Pruned` /
///   `ShiftedIdle`
/// - `anchor_pts` (`Stats`): number of anchors (constant world points of marginalized keyframes,
///   `backend.marginalization_anchor`) per cluster, over the clusters with at least one anchor, at the end of the
///   keyframe. Empty (`n` = 0) when the switch is off.
///
/// Fixed variables of iSAM2 after the smoother update, recorded at every keyframe:
/// - `n_fixed_kf`: number of keyframes in the sliding window whose pose key `X(k)` is in
///   `ISAM2::getFixedVariables()` after the update. iSAM2 never relinearizes these
///   variables again until they are marginalized, so a stale linearization point stays in the smoother.
///   Expected: 1 (`X(k+1)` of the IMU factor).
/// - `n_window_kf`: number of keyframes in the sliding window after the update (tells if `n_fixed_kf` is the whole
///   window)
/// - `fixed_kf_first`, `fixed_kf_last`: smallest and largest keyframe index `k` whose `X(k)` is in
///   `ISAM2::getFixedVariables()`, `nan` when none is fixed
/// - `n_fixed_vb`: number of window keyframes whose velocity key `V(k)` or bias key `B(k)` is fixed
///
/// Marginalization, `nan` at keyframes without a marginalization. If one update marginalizes more than one keyframe
/// (e.g. after a gap), each value covers all of them together:
/// - `marg_n`: custom marginalization factors that `removeKeyframeFromClusters` creates
///   (`PointToPlaneFactor::createMarginalizationFactor`, unary on the marginalized pose)
/// - `marg_info_rot`, `marg_info_trans`: smallest eigenvalue of the rotation and translation blocks of the summed
///   whitened information `sum(H^T H)` of these factors (incl. the robust kernel weight). With more than one
///   marginalized keyframe, the blocks of the different poses are added into one matrix.
/// - `marg_w`: robust kernel weight of the source `PointToPlaneFactor` of each custom marginalization factor
/// - `marg_kf`: the oldest keyframe that the smoother marginalizes in this update
/// - Rebuild comparison: does the rebuild of a cluster factor without the marginalized keyframe weaken the rows that
///   stay on the other poses? For each cluster whose factor on a marginalized keyframe is removed and rebuilt in this
///   update: the old factor is linearized before `removeKeyframeFromClusters`, the rebuilt factor (read after the
///   smoother update) at the same values: the smoother estimate before the update, plus the IMU-propagated pose of
///   the new keyframe. The whitened row block of pose `X(j)` is
///   `A_j = sqrt(w) / sigma * J_j` (`w` robust kernel weight, `sigma` noise sigma, `J_j` unwhitened Jacobian).
///   Only the pose keys in both factors count (not the marginalized keyframes, not the new keyframe).
///   Clusters with a kernel weight of 0 are skipped.
///   - `rebuild_n`: compared clusters
///   - `rebuild_info_ratio`: `sum ||A'_j||^2 / sum ||A_j||^2` over all compared clusters and their common poses
///     (after / before). Below 1: the rebuilt factors carry less information on the remaining poses.
///   - `rebuild_jac_ratio`: per cluster `||J'|| / ||J||` on the common poses (plane refit, residual `r_j`, `1/N`)
///   - Split by cluster state at the smoother update (after `createAndUpdateFactors`). Both groups get a plane refit
///     in `LidarFrontend::trackScanPointsToClusters` after the marginalized point is removed. `Tracked`: the track
///     succeeded, the plane is refit with the point of the new keyframe. `Idle` ("shifted"): the track failed, the
///     plane is refit from the remaining points only (`updateClusterParameters` without a new point), and
///     `createAndUpdateFactors` sets `ShiftedIdle` back to `Idle`. A cluster in any other state counts in the totals
///     only.
///     - `rebuild_n_tracked`, `rebuild_n_shifted`: compared clusters of the group
///     - `rebuild_info_ratio_tracked`, `rebuild_info_ratio_shifted`: `rebuild_info_ratio` of the group, `nan` if the
///       group is empty
///     - `rebuild_jac_ratio_tracked`, `rebuild_jac_ratio_shifted`: `rebuild_jac_ratio` of the group
///   - `rebuild_n_collapsed`: compared clusters with `||J'|| / ||J|| < 1e-6` (collapsed)
///   - `rebuild_collapsed_pts`: per collapsed cluster the number of pose keys `X(j)` of the rebuilt factor
///     (= points, one per keyframe)
///   - `rebuild_marg_share`: share of the cluster information that sits on the marginalized keyframe (the part that
///     goes into the unary factor). `sum_c w/sigma^2 ||J_c,k||^2 / sum_c w/sigma^2 (||J_c,k||^2 + sum_j ||J_c,j||^2)`
///     over the compared clusters `c` (old factor), `k` the marginalized pose keys, `j` the common poses of the ratios.
///     `nan` if the denominator is 0.
///   - `rebuild_sigma_ratio`: per cluster `sigma / sigma'` (noise model, plane thickness history)
///   - `rebuild_w_ratio`: per cluster `sqrt(w' / w)` (robust kernel)
///   The product of the three per-cluster factors is the per-cluster row ratio `||A'|| / ||A||`.
#define KF_TELEMETRY_FIELDS(X)                                                                                         \
    X(Index, kf)                                                                                                       \
    X(Time, t)                                                                                                         \
    X(Flag, lost_tracking)                                                                                             \
    X(Real, trigger_dist)                                                                                              \
    X(Real, trigger_angle)                                                                                             \
    X(Real, trigger_dt)                                                                                                \
    X(Real, imu_pred_dpos)                                                                                             \
    X(Real, imu_pred_drot)                                                                                             \
    X(Real, vel_norm)                                                                                                  \
    X(Real, ba_norm)                                                                                                   \
    X(Real, bg_norm)                                                                                                   \
    X(Index, pts_submap)                                                                                               \
    X(Index, clusters_valid)                                                                                           \
    X(Stats, thickness)                                                                                                \
    X(Real, eig_rot_min)                                                                                               \
    X(Real, eig_trans_min)                                                                                             \
    X(Stats, err_imu)                                                                                                  \
    X(Stats, err_lidar)                                                                                                \
    X(Stats, w_lidar)                                                                                                  \
    X(Stats, lidar_var)                                                                                                \
    X(Real, cov_rot_trace)                                                                                             \
    X(Real, cov_trans_trace)                                                                                           \
    X(Index, lidar_fac_before)                                                                                         \
    X(Index, lidar_fac_after)                                                                                          \
    X(Index, lidar_fac_removed)                                                                                        \
    X(Index, lidar_fac_readded)                                                                                        \
    X(Index, lidar_fac_new)                                                                                            \
    X(Index, lidar_fac_marg_pruned)                                                                                    \
    X(Index, lidar_fac_marg_lost)                                                                                      \
    X(Index, clusters_pruned)                                                                                          \
    X(Index, clusters_shifted_idle)                                                                                    \
    X(Index, n_fixed_kf)                                                                                               \
    X(Index, n_window_kf)                                                                                              \
    X(Real, fixed_kf_first)                                                                                            \
    X(Real, fixed_kf_last)                                                                                             \
    X(Index, n_fixed_vb)                                                                                               \
    X(Stats, anchor_pts)                                                                                               \
    X(Index, marg_n)                                                                                                   \
    X(Real, marg_info_rot)                                                                                             \
    X(Real, marg_info_trans)                                                                                           \
    X(Stats, marg_w)                                                                                                   \
    X(Index, marg_kf)                                                                                                  \
    X(Index, rebuild_n)                                                                                                \
    X(Index, rebuild_n_tracked)                                                                                        \
    X(Index, rebuild_n_shifted)                                                                                        \
    X(Real, rebuild_info_ratio)                                                                                        \
    X(Real, rebuild_info_ratio_tracked)                                                                                \
    X(Real, rebuild_info_ratio_shifted)                                                                                \
    X(Stats, rebuild_jac_ratio)                                                                                        \
    X(Stats, rebuild_jac_ratio_tracked)                                                                                \
    X(Stats, rebuild_jac_ratio_shifted)                                                                                \
    X(Stats, rebuild_sigma_ratio)                                                                                      \
    X(Stats, rebuild_w_ratio)                                                                                          \
    X(Index, rebuild_n_collapsed)                                                                                      \
    X(Stats, rebuild_collapsed_pts)                                                                                    \
    X(Real, rebuild_marg_share)

namespace mapping
{
    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Field types of `KF_TELEMETRY_FIELDS`, each with its value type, initial value and CSV writers.
        namespace field
        {
            constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

            /// @brief Distribution of a sample. With `n = 0` all other values are `nan`.
            struct StatsValue
            {
                std::uint64_t n = 0;
                double min = NaN, max = NaN, median = NaN, mean = NaN, std = NaN;
            };

            struct Index
            {
                using value_type = std::optional<std::uint64_t>;
                static value_type init() { return std::nullopt; }
                static void header(std::string &out, const char *name);
                static void write(std::string &out, const value_type &value, double bagStart);
            };

            struct Flag
            {
                using value_type = std::uint8_t;
                static value_type init() { return 0; }
                static void header(std::string &out, const char *name);
                static void write(std::string &out, const value_type &value, double bagStart);
            };

            struct Real
            {
                using value_type = double;
                static value_type init() { return NaN; }
                static void header(std::string &out, const char *name);
                static void write(std::string &out, const value_type &value, double bagStart);
            };

            struct Time
            {
                /// @brief System time `[s]`, relative to the first bag message.
                using value_type = double;
                static value_type init() { return NaN; }
                static void header(std::string &out, const char *name);
                static void write(std::string &out, const value_type &value, double bagStart);
            };

            struct Stats
            {
                using value_type = StatsValue;
                static value_type init() { return {}; }
                static void header(std::string &out, const char *name);
                static void write(std::string &out, const value_type &value, double bagStart);
            };
        } // namespace field

#define KF_TELEMETRY_MEMBER(Type, name) field::Type::value_type name = field::Type::init();

        /// @ingroup benchmarking
        /// @brief One row of `telemetry.csv`, members generated from `KF_TELEMETRY_FIELDS`.
        struct KeyframeTelemetry
        {
            KF_TELEMETRY_FIELDS(KF_TELEMETRY_MEMBER)
        };

#undef KF_TELEMETRY_MEMBER

        /// @ingroup benchmarking
        /// @brief Fill a `Stats` value from a sample: count, min, max, median, mean and population standard deviation.
        field::StatsValue makeStats(std::vector<double> samples);

        /// @ingroup benchmarking
        /// @brief Append the CSV header line (incl. newline).
        void appendCsvHeader(std::string &out);

        /// @ingroup benchmarking
        /// @brief Append one CSV row (incl. newline).
        /// @param bagStart Absolute bag time of the first message, added to `Time` fields.
        void appendCsvRow(std::string &out, const KeyframeTelemetry &row, double bagStart);
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_KEYFRAMETELEMETRY_HPP_
