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
/// - `voxel_size`: adaptive voxel size when the keyframe is created (`States::getVoxelSize()`)
/// - `clusters_valid`: `numValidClusters` of `LidarFrontend::trackScanPointsToClusters` at the current keyframe,
///   `nan` when it returned false (see `LidarFrontend::keyframeNumValidClusters`)
/// - `thickness`: newest plane thickness history entry of the `Tracked` clusters with a point in the current
///   keyframe. `thickness_n` can be slightly larger than `clusters_valid` (a cluster on the `clusterTooSmall`
///   path also gets a point in the current keyframe but is not counted)
/// - `eig_rot_min`, `eig_trans_min`: smallest eigenvalue of the rotation and translation blocks
///   of the point-to-plane information matrix of this keyframe pose
/// - `err_imu`, `err_lidar`: per-factor error of all `CombinedImuFactor` / `PointToPlaneFactor` in the smoother.
///   `err_lidar` is `0.5 * |b|^2` of `PointToPlaneFactor::linearize`, the system the optimizer solves.
///   `PointToPlaneFactor::unwhitenedError` throws by design and `evaluateErrorDiagnostic` has no definition.
///   With a robust noise model `b` is scaled by `sqrt(w(r))` (IRLS weight), so this is `0.5 * w(r) * r^2`,
///   not the robust loss `rho(r)`. For large residuals the value falls towards 0 instead of saturating.
///   Read a small `err_lidar` together with `clusters_valid` and `thickness` before trusting it.
/// - `err_marg_sum`: summed error of all `LinearContainerFactor` (marginalization) in the smoother
/// - `cov_rot_trace`, `cov_trans_trace`: trace of the rotation and translation blocks of the marginal covariance
///   of `X(kf)`
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
    X(Real, voxel_size)                                                                                                \
    X(Index, clusters_valid)                                                                                           \
    X(Stats, thickness)                                                                                                \
    X(Real, eig_rot_min)                                                                                               \
    X(Real, eig_trans_min)                                                                                             \
    X(Stats, err_imu)                                                                                                  \
    X(Stats, err_lidar)                                                                                                \
    X(Real, err_marg_sum)                                                                                              \
    X(Real, cov_rot_trace)                                                                                             \
    X(Real, cov_trans_trace)

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
