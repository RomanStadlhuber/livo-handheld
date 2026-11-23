/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_RUNOUTPUT_HPP_
#define MAPPING_BENCHMARKING_RUNOUTPUT_HPP_

#include <mapping/Config.hpp>
#include <mapping/benchmarking/TelemetryRecorder.hpp>
#include <mapping/benchmarking/TrajectoryRecorder.hpp>

#include <cstddef>
#include <string>

namespace mapping
{
    /// @ingroup benchmarking
    /// @brief Inputs and results of a bag run, written to `run.yaml`.
    struct RunInfo
    {
        std::string bagPath;
        std::string topicImu;
        std::string topicLidar;
        double scalingImu = 1.0;
        /// @brief Config path as given on the command line.
        std::string configPath;
        /// @brief Absolute bag time `[s]` of the first message. Added to relative keyframe times,
        /// this is the single place defining the offset for all benchmark output files.
        double bagStart = 0.0;
        /// @brief Wall-clock start of the run, ISO 8601 (UTC).
        std::string wallStart;
        double wallDurationS = 0.0;
        double realtimeFactor = 0.0;
        int exitCode = 0;
        size_t msgsImu = 0;
        size_t msgsLidar = 0;
        size_t scansSkipped = 0;
        size_t keyframes = 0;
    };

    /// @ingroup benchmarking
    /// @brief Current wall-clock time as ISO 8601 string (UTC).
    std::string wallClockNowISO();

    /// @ingroup benchmarking
    /// @brief Create the run directory. Fails if it exists and is not empty, so earlier runs are never overwritten.
    /// @param error Set to a message on failure.
    bool prepareRunDir(const std::string &dir, std::string &error);

    /// @ingroup benchmarking
    /// @brief Write `trajectory.tum`, `telemetry.csv`, `config.yaml` and `run.yaml` into the run directory.
    /// @details `config.yaml` is the serialized `config` as used by the system, not a copy of the user file.
    /// It holds the real values, incl. defaults for keys missing in the user file.
    /// `run.yaml` is written even if another file fails. In that case it records exit code 1.
    /// @param error Set to a message on failure.
    /// @return false if any file could not be written.
    bool writeRunOutput(const std::string &dir, const TrajectoryRecorder &trajectory,
                        const benchmarking::TelemetryRecorder &telemetry, const MappingConfig &config, RunInfo info,
                        std::string &error);
} // namespace mapping

#endif // MAPPING_BENCHMARKING_RUNOUTPUT_HPP_
