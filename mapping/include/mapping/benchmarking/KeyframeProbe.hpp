/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_
#define MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_

#include <mapping/benchmarking/Instrumentation.hpp>
#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <gtsam/geometry/Pose3.h>

#include <cstdint>

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
        ///   and of the marginalization reference (factors and estimate before the update). Never throws.
        /// - destructor: runs on every exit of `track()`, also the early lost-tracking return.
        ///   Collects the marginalization columns (skipped when the smoother update of this keyframe did not run, or
        ///   the snapshot failed) and the post-update columns, and
        ///   pushes the row into the telemetry queue of the system,
        ///   see `MappingSystem::takeTelemetry()`. Never throws.
        ///
        /// Reads the private members of `MappingSystem` as a friend, const access only.
        class KeyframeProbe
        {
        public:
            KeyframeProbe(MappingSystem &system, uint32_t idxKeyframe) noexcept;
            ~KeyframeProbe();

            KeyframeProbe(const KeyframeProbe &) = delete;
            KeyframeProbe &operator=(const KeyframeProbe &) = delete;

        private:
            MappingSystem &system_;
            KeyframeTelemetry row_;
            gtsam::Pose3 imuPropagated_;
            /// @brief The marginalization reference before the update, see `rebuild_*`.
            instrumentation::MarginalizationSnapshot marginalization_;
            /// @brief False when the snapshot step threw, the marginalization columns then stay `nan`.
            bool marginalizationValid_ = true;
        };
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_KEYFRAMEPROBE_HPP_
