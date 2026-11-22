/// @file
/// @ingroup benchmarking
#include <mapping/benchmarking/BuildInfo.hpp>
#include <mapping/benchmarking/RunOutput.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>

namespace fs = std::filesystem;

namespace mapping
{
    namespace
    {
        /// @brief Quote a string as YAML double-quoted scalar.
        std::string yamlQuote(const std::string &s)
        {
            std::string out = "\"";
            for (const char c : s)
            {
                if (c == '"' || c == '\\')
                    out += '\\';
                if (c == '\n')
                    out += "\\n";
                else
                    out += c;
            }
            return out + '"';
        }

        bool writeRunYaml(const fs::path &path, const RunInfo &info, size_t keyframesWritten)
        {
            std::ofstream file(path);
            if (!file)
                return false;
            std::error_code ec;
            const fs::path bagAbs = fs::absolute(info.bagPath, ec);
            const int dirty = gitDirty();
            file << "git:\n"
                 << "  commit: " << yamlQuote(gitCommit()) << '\n'
                 << "  dirty: " << (dirty < 0 ? "null" : (dirty ? "true" : "false")) << '\n'
                 << "build:\n"
                 << "  type: " << yamlQuote(buildType()) << '\n'
                 << "  benchmarking: true\n"
                 << "input:\n"
                 << "  bag: " << yamlQuote(ec ? info.bagPath : bagAbs.string()) << '\n'
                 << "  topic_imu: " << yamlQuote(info.topicImu) << '\n'
                 << "  topic_lidar: " << yamlQuote(info.topicLidar) << '\n'
                 << "  scaling_imu: " << info.scalingImu << '\n'
                 << "  config: " << yamlQuote(info.configPath) << '\n'
                 << "time:\n"
                 << std::fixed << std::setprecision(9) << "  bag_start: " << info.bagStart << '\n'
                 << std::defaultfloat << "  wall_start: " << yamlQuote(info.wallStart) << '\n'
                 << "  wall_duration_s: " << info.wallDurationS << '\n'
                 << "  realtime_factor: " << info.realtimeFactor << '\n'
                 << "result:\n"
                 << "  exit_code: " << info.exitCode << '\n'
                 << "  msgs_imu: " << info.msgsImu << '\n'
                 << "  msgs_lidar: " << info.msgsLidar << '\n'
                 << "  scans_skipped: " << info.scansSkipped << '\n'
                 << "  keyframes: " << info.keyframes << '\n'
                 << "  keyframes_written: " << keyframesWritten << '\n';
            file.flush();
            return file.good();
        }

        /// @brief Append a message to the error string, so that no earlier failure is lost.
        void appendError(std::string &error, const std::string &message)
        {
            if (!error.empty())
                error += "; ";
            error += message;
        }
    } // namespace

    std::string wallClockNowIso()
    {
        const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
        gmtime_r(&t, &tm);
        std::ostringstream ss;
        ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }

    bool prepareRunDir(const std::string &dir, std::string &error)
    {
        std::error_code ec;
        if (fs::exists(dir, ec))
        {
            if (!fs::is_directory(dir, ec))
            {
                error = "output path '" + dir + "' exists and is not a directory";
                return false;
            }
            if (!fs::is_empty(dir, ec))
            {
                error = "output directory '" + dir + "' is not empty, refusing to overwrite an earlier run";
                return false;
            }
            return true;
        }
        if (!fs::create_directories(dir, ec) || ec)
        {
            error = "failed to create output directory '" + dir + "': " + ec.message();
            return false;
        }
        return true;
    }

    bool writeRunOutput(const std::string &dir, const TrajectoryRecorder &recorder, RunInfo info, std::string &error)
    {
        const fs::path root(dir);
        bool ok = true;
        if (!recorder.writeTum((root / "trajectory.tum").string(), info.bagStart))
        {
            appendError(error, "failed to write '" + (root / "trajectory.tum").string() + "'");
            ok = false;
        }
        std::error_code ec;
        fs::copy_file(info.configPath, root / "config.yaml", fs::copy_options::overwrite_existing, ec);
        if (ec)
        {
            appendError(error, "failed to copy config '" + info.configPath + "': " + ec.message());
            ok = false;
        }
        if (!ok)
            info.exitCode = 1;
        if (!writeRunYaml(root / "run.yaml", info, recorder.size()))
        {
            appendError(error, "failed to write '" + (root / "run.yaml").string() + "'");
            ok = false;
        }
        return ok;
    }
} // namespace mapping
