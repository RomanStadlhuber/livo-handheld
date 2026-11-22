/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_BUILDINFO_HPP_
#define MAPPING_BENCHMARKING_BUILDINFO_HPP_

namespace mapping
{
    /// @ingroup benchmarking
    /// @brief Git commit hash of the source tree at build time, `unknown` if git was not available.
    const char *gitCommit();

    /// @ingroup benchmarking
    /// @brief Whether the source tree had uncommitted changes at build time: 1 = dirty, 0 = clean, -1 = unknown.
    int gitDirty();

    /// @ingroup benchmarking
    /// @brief CMake build type, e.g. `Release`, or `unspecified`.
    const char *buildType();
} // namespace mapping

#endif // MAPPING_BENCHMARKING_BUILDINFO_HPP_
