/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
#define MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_

/// @brief Home of the benchmark call-point macros, like `LOG` in `logging.hpp`.
/// @details Each macro has two definitions: the real one under `MAPPING_BENCHMARKING`,
/// and one that expands to nothing otherwise. So the production binary has no telemetry code
/// and no runtime cost.
#ifdef MAPPING_BENCHMARKING

#include <mapping/benchmarking/KeyframeProbe.hpp>

/// @brief Record the telemetry row of a new keyframe when the enclosing scope exits.
/// @details Place it in `MappingSystem::track()` right after the keyframe is created, see `KeyframeProbe`.
#define BENCH_KEYFRAME_PROBE(idxKeyframe)                                                                              \
    const ::mapping::benchmarking::KeyframeProbe benchKeyframeProbe_ { *this, (idxKeyframe) }

#else

#define BENCH_KEYFRAME_PROBE(idxKeyframe)

#endif

#endif // MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
