/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_TRAJECTORYRECORDER_HPP_
#define MAPPING_BENCHMARKING_TRAJECTORYRECORDER_HPP_

#include <mapping/types.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace mapping
{
    /// @ingroup benchmarking
    /// @brief Records the keyframe trajectory for offline evaluation, e.g. with evo.
    /// @details Keeps the latest estimate of every keyframe. While a keyframe is in the sliding window,
    /// each update replaces its pose, so the stored pose is the last estimate before marginalization.
    class TrajectoryRecorder
    {
    public:
        /// @brief Store the keyframe states of the current sliding window.
        /// @param states Sliding window states, see `MappingSystem::getStates()`.
        /// @param keyframeCount Number of keyframes created so far, see `MappingSystem::getKeyframeCount()`.
        /// States with an index >= keyframeCount are IMU-propagated states, not keyframes, and are ignored.
        void update(const SlidingWindowStates &states, uint32_t keyframeCount);

        /// @brief Write the trajectory in TUM format, one row per keyframe: `timestamp x y z qx qy qz qw`.
        /// @details Poses are the IMU (body) frame in the world frame.
        /// @param path Output file path, overwritten if it exists.
        /// @param timeOffset Added to every timestamp, e.g. to convert relative times back to absolute times.
        /// @return false if the file could not be written.
        bool writeTum(const std::string &path, double timeOffset = 0.0) const;

        /// @brief Number of recorded keyframes.
        size_t size() const { return keyframes_.size(); }

    private:
        SlidingWindowStates keyframes_;
    };
} // namespace mapping

#endif // MAPPING_BENCHMARKING_TRAJECTORYRECORDER_HPP_
