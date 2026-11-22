/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
#define MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_

/// @brief Home of the benchmark call-point macros, like `LOG` in `logging.hpp`.
/// @details Each macro has two definitions: the real one under `MAPPING_BENCHMARKING`,
/// and one that expands to nothing otherwise. So the production binary has no telemetry code
/// and no runtime cost. No macro is defined yet. The telemetry follow-up adds `BENCH_KEYFRAME_PROBE`,
/// the single call point in `MappingSystem::track()`.
#ifdef MAPPING_BENCHMARKING

// benchmark build: call-point macros forward to the benchmarking module

#else

// production build: the same macros expand to nothing

#endif

#endif // MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
