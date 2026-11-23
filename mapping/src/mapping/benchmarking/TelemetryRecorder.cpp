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
    } // namespace benchmarking
} // namespace mapping
