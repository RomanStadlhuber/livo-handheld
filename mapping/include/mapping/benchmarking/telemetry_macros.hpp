/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
#define MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_

/// @brief Call-point macros for benchmark telemetry, like `LOG` in `logging.hpp`.
/// @details Expand to nothing unless built with `-DMAPPING_BENCHMARKING=ON`, so the production
/// binary contains no telemetry code and pays no runtime cost.
/// With benchmarking on, the macros forward to the stub functions below. They do not record anything yet.
/// Replace the stubs with a real telemetry sink when the telemetry signals are added.
#ifdef MAPPING_BENCHMARKING

namespace mapping
{
    namespace benchmarking::detail
    {
        /// stub sink for a named scalar value, replaced by the telemetry writer
        template <typename T> inline void telemetryValue(const char * /*name*/, const T & /*value*/) {}

        /// stub RAII guard for a named timed scope, replaced by the telemetry writer
        struct TelemetryScope
        {
            explicit TelemetryScope(const char * /*name*/) {}
        };
    } // namespace benchmarking::detail
} // namespace mapping

#define TELEMETRY_CONCAT_IMPL(a, b) a##b
#define TELEMETRY_CONCAT(a, b) TELEMETRY_CONCAT_IMPL(a, b)

/// @brief Record a named scalar value at this call point.
#define TELEMETRY_VALUE(name, value)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        mapping::benchmarking::detail::telemetryValue((name), (value));                                                \
    } while (0)

/// @brief Time the enclosing scope under the given name.
#define TELEMETRY_SCOPE(name)                                                                                          \
    const mapping::benchmarking::detail::TelemetryScope TELEMETRY_CONCAT(_telemetryScope, __COUNTER__)(name)

#else

#define TELEMETRY_VALUE(name, value)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
    } while (0)
#define TELEMETRY_SCOPE(name)

#endif

#endif // MAPPING_BENCHMARKING_TELEMETRY_MACROS_HPP_
