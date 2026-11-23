/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/Format.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace mapping
{
    namespace benchmarking
    {
        void appendBagTime(std::string &out, double tRelative, double bagStart)
        {
            const double t = tRelative + bagStart;
            if (std::isnan(t))
            {
                out += "nan";
                return;
            }
            char buf[64];
            const int n = std::snprintf(buf, sizeof(buf), "%.9f", t);
            if (n > 0)
                out.append(buf, static_cast<std::size_t>(std::min(n, static_cast<int>(sizeof(buf)) - 1)));
        }

        void appendReal(std::string &out, double value)
        {
            if (std::isnan(value))
            {
                out += "nan";
                return;
            }
            char buf[64];
            const std::to_chars_result res = std::to_chars(buf, buf + sizeof(buf), value);
            out.append(buf, res.ptr);
        }
    } // namespace benchmarking
} // namespace mapping
