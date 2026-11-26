/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/TelemetryRecorder.hpp>

#include <fstream>
#include <iterator>

namespace mapping
{
    namespace benchmarking
    {
        void TelemetryRecorder::append(std::vector<KeyframeTelemetry> rows)
        {
            rows_.insert(rows_.end(), std::make_move_iterator(rows.begin()), std::make_move_iterator(rows.end()));
        }

        bool TelemetryRecorder::writeCsv(const std::string &path, double bagStart) const
        {
            std::ofstream file(path);
            if (!file)
                return false;
            std::string line;
            appendCsvHeader(line);
            file << line;
            for (const KeyframeTelemetry &row : rows_)
            {
                line.clear();
                appendCsvRow(line, row, bagStart);
                file << line;
            }
            file.flush();
            return file.good();
        }

        namespace
        {
            /// @brief Write a header and one line per detail row of each keyframe.
            template <typename Header, typename Rows, typename Write>
            bool writeDetailCsv(const std::string &path, const std::vector<KeyframeTelemetry> &keyframes,
                                Header &&header, Rows &&rows, Write &&write)
            {
                std::ofstream file(path);
                if (!file)
                    return false;
                std::string line;
                header(line);
                file << line;
                for (const KeyframeTelemetry &keyframe : keyframes)
                    for (const auto &row : rows(keyframe))
                    {
                        line.clear();
                        write(line, row);
                        file << line;
                    }
                file.flush();
                return file.good();
            }
        } // namespace

        bool TelemetryRecorder::writeAnchorsCsv(const std::string &path) const
        {
            return writeDetailCsv(
                path, rows_, appendAnchorsCsvHeader,
                [](const KeyframeTelemetry &kf) -> const auto & { return kf.anchors; }, appendAnchorsCsvRow);
        }

        bool TelemetryRecorder::writeClustersCsv(const std::string &path) const
        {
            return writeDetailCsv(
                path, rows_, appendClustersCsvHeader,
                [](const KeyframeTelemetry &kf) -> const auto & { return kf.clusterSpans; }, appendClustersCsvRow);
        }
    } // namespace benchmarking
} // namespace mapping
