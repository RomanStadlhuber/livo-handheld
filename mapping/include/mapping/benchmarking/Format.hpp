/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_FORMAT_HPP_
#define MAPPING_BENCHMARKING_FORMAT_HPP_

#include <string>

namespace mapping
{
    /// @ingroup benchmarking
    /// @brief Append an absolute bag time as `%.9f`, e.g. `1649764528.063481092`.
    /// @details All benchmark output files write times through this function, so that a join on the time column
    /// finds equal values in all files. `nan` is written as `nan`.
    /// @param tRelative Time relative to the first bag message `[s]`, as used inside the system.
    /// @param bagStart Absolute bag time of the first message `[s]`, see `RunInfo::bagStart`.
    void appendBagTime(std::string &out, double tRelative, double bagStart);

    /// @ingroup benchmarking
    /// @brief Append a double in the shortest form that reads back to the same value (`std::to_chars`).
    /// @details Uses scientific notation when that is shorter, e.g. `1.6e-07`. `nan` is written as `nan`.
    void appendReal(std::string &out, double value);
} // namespace mapping

#endif // MAPPING_BENCHMARKING_FORMAT_HPP_
