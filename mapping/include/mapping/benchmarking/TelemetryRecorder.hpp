/// @file
/// @ingroup benchmarking
#pragma once

#ifndef MAPPING_BENCHMARKING_TELEMETRYRECORDER_HPP_
#define MAPPING_BENCHMARKING_TELEMETRYRECORDER_HPP_

#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace mapping
{
    namespace benchmarking
    {
        /// @ingroup benchmarking
        /// @brief Collects the keyframe telemetry rows of a run and writes `telemetry.csv`.
        class TelemetryRecorder
        {
        public:
            /// @brief Append rows, e.g. from `MappingSystem::takeTelemetry()`.
            void append(std::vector<KeyframeTelemetry> rows);

            /// @brief Write all rows as CSV, header from `KF_TELEMETRY_FIELDS`.
            /// @param path Output file path, overwritten if it exists.
            /// @param bagStart Absolute bag time of the first message, added to the time columns.
            /// @return false if the file could not be written.
            bool writeCsv(const std::string &path, double bagStart) const;

            /// @brief Number of recorded rows.
            std::size_t size() const { return rows_.size(); }

        private:
            std::vector<KeyframeTelemetry> rows_;
        };
    } // namespace benchmarking
} // namespace mapping

#endif // MAPPING_BENCHMARKING_TELEMETRYRECORDER_HPP_
