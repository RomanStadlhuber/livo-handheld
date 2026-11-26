/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/Format.hpp>
#include <mapping/benchmarking/KeyframeTelemetry.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace mapping
{
    namespace benchmarking
    {
        namespace field
        {
            void Index::header(std::string &out, const char *name) { (out += name) += ','; }

            void Index::write(std::string &out, const value_type &value, double)
            {
                out += value ? std::to_string(*value) : std::string("nan");
                out += ',';
            }

            void Flag::header(std::string &out, const char *name) { (out += name) += ','; }

            void Flag::write(std::string &out, const value_type &value, double) { out += value ? "1," : "0,"; }

            void Real::header(std::string &out, const char *name) { (out += name) += ','; }

            void Real::write(std::string &out, const value_type &value, double)
            {
                appendReal(out, value);
                out += ',';
            }

            void Time::header(std::string &out, const char *name) { (out += name) += ','; }

            void Time::write(std::string &out, const value_type &value, double bagStart)
            {
                appendBagTime(out, value, bagStart);
                out += ',';
            }

            void Stats::header(std::string &out, const char *name)
            {
                for (const char *suffix : {"_n,", "_min,", "_max,", "_median,", "_mean,", "_std,"})
                    (out += name) += suffix;
            }

            void Stats::write(std::string &out, const value_type &value, double)
            {
                out += std::to_string(value.n);
                for (const double v : {value.min, value.max, value.median, value.mean, value.std})
                {
                    out += ',';
                    appendReal(out, value.n > 0 ? v : NaN);
                }
                out += ',';
            }
        } // namespace field

        field::StatsValue makeStats(std::vector<double> samples)
        {
            field::StatsValue stats;
            stats.n = samples.size();
            if (samples.empty())
                return stats;
            std::sort(samples.begin(), samples.end());
            const std::size_t n = samples.size();
            stats.min = samples.front();
            stats.max = samples.back();
            stats.median = n % 2 == 1 ? samples[n / 2] : 0.5 * (samples[n / 2 - 1] + samples[n / 2]);
            stats.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(n);
            double sumSq = 0.0;
            for (const double v : samples)
                sumSq += (v - stats.mean) * (v - stats.mean);
            stats.std = std::sqrt(sumSq / static_cast<double>(n));
            return stats;
        }

        void appendCsvHeader(std::string &out)
        {
#define KF_TELEMETRY_HEADER(Type, name) field::Type::header(out, #name);
            KF_TELEMETRY_FIELDS(KF_TELEMETRY_HEADER)
#undef KF_TELEMETRY_HEADER
            out.back() = '\n'; // replace the trailing separator
        }

        void appendCsvRow(std::string &out, const KeyframeTelemetry &row, double bagStart)
        {
#define KF_TELEMETRY_VALUE(Type, name) field::Type::write(out, row.name, bagStart);
            KF_TELEMETRY_FIELDS(KF_TELEMETRY_VALUE)
#undef KF_TELEMETRY_VALUE
            out.back() = '\n'; // replace the trailing separator
        }

        void appendClustersCsvHeader(std::string &out) { out += "kf,cluster_id,keyframes,info,weight,touches_marg\n"; }

        void appendClustersCsvRow(std::string &out, const ClusterSpanRow &row)
        {
            (((out += std::to_string(row.kf)) += ',') += std::to_string(row.clusterId)) += ',';
            for (std::size_t i = 0; i < row.keyframes.size(); i++)
            {
                if (i > 0)
                    out += ' ';
                out += std::to_string(row.keyframes[i]);
            }
            out += ',';
            appendReal(out, row.info);
            out += ',';
            appendReal(out, row.weight);
            out += row.touchesMarg ? ",1\n" : ",0\n";
        }
    } // namespace benchmarking
} // namespace mapping
