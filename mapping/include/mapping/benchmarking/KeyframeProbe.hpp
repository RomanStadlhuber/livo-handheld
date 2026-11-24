/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_
#define MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_

#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstdint>
#include <vector>

namespace mapping
{
    class MappingSystem;

    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Records the telemetry row of one keyframe, placed in `MappingSystem::track()` by
        /// `BENCH_KEYFRAME_PROBE` right after the keyframe is created.
        /// @details
        /// - constructor: snapshot of the values that the update overwrites (IMU-propagated pose, trigger values)
        /// - `beforeMarginalization()`, `beforeSmootherUpdate()`: marginalization columns, called by
        ///   `BENCH_BEFORE_MARGINALIZATION` / `BENCH_BEFORE_SMOOTHER_UPDATE`. Never throw.
        /// - destructor: runs on every exit of `track()`, also the early lost-tracking return.
        ///   Collects the post-update columns and pushes the row into the telemetry queue of the system,
        ///   see `MappingSystem::takeTelemetry()`. Never throws.
        ///
        /// Reads the private members of `MappingSystem` as a friend, const access only.
        class KeyframeProbe
        {
        public:
            KeyframeProbe(MappingSystem &system, uint32_t idxKeyframe) noexcept;
            ~KeyframeProbe();

            /// @brief Call before the keyframes outside the sliding window are removed from the clusters.
            void beforeMarginalization() noexcept;
            /// @brief Call right before the smoother update with the LiDAR factors that it adds and removes.
            void beforeSmootherUpdate(const gtsam::NonlinearFactorGraph &newFactors,
                                      const gtsam::FactorIndices &factorsToRemove) noexcept;

            KeyframeProbe(const KeyframeProbe &) = delete;
            KeyframeProbe &operator=(const KeyframeProbe &) = delete;

        private:
            MappingSystem &system_;
            KeyframeTelemetry row_;
            gtsam::Pose3 imuPropagated_;
            /// @brief Keyframes that the smoother marginalizes in the update of this keyframe.
            std::vector<uint32_t> marginalizedKeyframes_;
        };
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_
