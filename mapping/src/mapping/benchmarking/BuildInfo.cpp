/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/BuildInfo.hpp>

// generated at build time by cmake/build_info.cmake, only included here to limit recompilation
#include <mapping/build_info.hpp>

#ifndef MAPPING_BUILD_TYPE
#define MAPPING_BUILD_TYPE ""
#endif

namespace mapping
{
    const char *gitCommit() { return MAPPING_GIT_COMMIT; }

    int gitDirty() { return MAPPING_GIT_DIRTY; }

    const char *buildType() { return MAPPING_BUILD_TYPE[0] != '\0' ? MAPPING_BUILD_TYPE : "unspecified"; }
} // namespace mapping
