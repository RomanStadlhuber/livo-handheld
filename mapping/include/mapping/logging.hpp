/// @file
/// @ingroup logging
#pragma once

#ifndef MAPPING_LOGGING_HPP_
#define MAPPING_LOGGING_HPP_

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <unistd.h>

// ANSI color codes for console output
// from https://gist.github.com/Kielx/2917687bc30f567d45e15a4577772b02
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLD "\033[1m"                /* Bold (no color) */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

namespace mapping
{

    /// @ingroup logging
    /// @brief Log severity levels. Lower values are more verbose.
    /// Set to NONE to suppress all output.
    enum class LogLevel
    {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        NONE = 4
    };

    namespace detail
    {

        /// shared log file, opened by ENABLE_FILE_LOGGING() and closed by CLOSE_LOGS()
        inline std::ofstream logFile;

        /// path of the currently open log file, printed by CLOSE_LOGS()
        inline std::string logFilePath;

        /// global minimum log level — messages below this threshold are suppressed regardless of module level
        inline LogLevel logLevel = LogLevel::DEBUG;

        /// returns true once and caches the result — avoids repeated isatty calls on the hot path
        inline bool colorEnabled()
        {
            static const bool enabled = isatty(STDOUT_FILENO);
            return enabled;
        }

        /// returns the display tag string for a given level
        inline const char *levelTag(LogLevel level)
        {
            switch (level)
            {
            case LogLevel::DEBUG:
                return "DEBUG";
            case LogLevel::INFO:
                return "INFO";
            case LogLevel::WARN:
                return "WARNING";
            case LogLevel::ERROR:
                return "ERROR";
            default:
                return "";
            }
        }

        /// returns the ANSI bold+color prefix for a given level
        inline const char *levelColor(LogLevel level)
        {
            switch (level)
            {
            case LogLevel::DEBUG:
                return BOLDBLUE;
            case LogLevel::INFO:
                return BOLD;
            case LogLevel::WARN:
                return BOLDYELLOW;
            case LogLevel::ERROR:
                return BOLDRED;
            default:
                return "";
            }
        }

        /// formats a timestamp as fixed-point with 4 decimal places
        inline std::string formatTimestamp(double ts)
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(4) << ts;
            return ss.str();
        }

        /// returns true if a message at msgLevel should be emitted given the module and global thresholds
        inline bool shouldLog(LogLevel msgLevel, LogLevel moduleLevel)
        {
            return msgLevel >= moduleLevel && msgLevel >= logLevel;
        }

        /// builds the plain-text prefix: "::: ModuleName [LEVEL]:" or "::: ModuleName [LEVEL] (ts):"
        inline std::string buildPrefix(const char *module, LogLevel level, std::optional<double> ts)
        {
            std::string prefix = "::: ";
            prefix += module;
            prefix += " [";
            prefix += levelTag(level);
            prefix += "]";
            if (ts.has_value())
            {
                prefix += " (";
                prefix += formatTimestamp(*ts);
                prefix += ")";
            }
            prefix += ":";
            return prefix;
        }

        /// writes a single formatted line to stdout and, if open, to the log file
        inline void writeLine(const std::string &line)
        {
            std::cout << line << "\n";
            if (logFile.is_open())
            {
                logFile << line << "\n";
                logFile.flush();
            }
        }

        /// core output function — formats and emits a log entry with an optional timestamp.
        /// The first element of lines is the header; subsequent elements are continuation lines
        /// indented by four spaces. Color applies only to the prefix on the header line.
        inline void logImpl(
            LogLevel level,
            const char *module,
            std::optional<double> ts,
            std::initializer_list<std::string> lines)
        {
            const std::string prefix = buildPrefix(module, level, ts);
            const bool useColor = colorEnabled();
            const char *color = levelColor(level);

            auto it = lines.begin();

            // header line — colored prefix followed by the message and closing :::
            {
                const std::string &msg = *it;
                if (useColor)
                    std::cout << color << prefix << RESET << " " << msg << " :::\n";
                else
                    std::cout << prefix << " " << msg << " :::\n";

                if (logFile.is_open())
                {
                    logFile << prefix << " " << msg << " :::\n";
                    logFile.flush();
                }
                ++it;
            }

            // continuation lines — indented, no prefix, no color
            for (; it != lines.end(); ++it)
                writeLine("    " + *it);
        }

        /// generates the timestamped log file path: /tmp/livo_YYYY_MM_DD_HH_mm_ss.log
        inline std::string generateLogFilePath()
        {
            const auto now = std::chrono::system_clock::now();
            const std::time_t t = std::chrono::system_clock::to_time_t(now);
            const std::tm tm = *std::localtime(&t);
            std::ostringstream ss;
            ss << "/tmp/livo_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".log";
            return ss.str();
        }

    } // namespace detail

    /// @ingroup logging
    /// @brief Set the global minimum log level threshold.
    /// Messages below this level are suppressed across all modules.
    /// Call once from MappingSystem based on the loaded configuration.
    inline void setLogLevel(LogLevel level)
    {
        detail::logLevel = level;
    }

} // namespace mapping

// --- per-module setup ---

/// @brief Declare a file-scope logger for this translation unit.
/// Place once at file scope in the .cpp file, before any logging calls.
/// @param level  Default log level for this module (DEBUG, INFO, WARN, ERROR, NONE).
/// @param name   Module name string shown in every log line from this file.
#define SETUP_LOGS(level, name)                     \
    static constexpr const char *_logModule = name; \
    static constexpr mapping::LogLevel _logModuleLevel = mapping::LogLevel::level;

// --- file logging lifecycle ---

/// @brief Open a timestamped log file at /tmp/livo_YYYY_MM_DD_HH_mm_ss.log.
/// All modules write to this single shared file. Call once at startup.
#define ENABLE_FILE_LOGGING()                                                  \
    do                                                                         \
    {                                                                          \
        mapping::detail::logFilePath = mapping::detail::generateLogFilePath(); \
        mapping::detail::logFile.open(                                         \
            mapping::detail::logFilePath, std::ios::app);                      \
    } while (0)

/// @brief Flush and close the log file, then print its path to the console.
/// Call on shutdown or SIGINT.
#define CLOSE_LOGS()                                                             \
    do                                                                           \
    {                                                                            \
        if (mapping::detail::logFile.is_open())                                  \
        {                                                                        \
            mapping::detail::logFile.flush();                                    \
            mapping::detail::logFile.close();                                    \
        }                                                                        \
        std::cout << "log written to: " << mapping::detail::logFilePath << "\n"; \
    } while (0)

// --- logging macros ---

/// @brief Emit a single-line log message.
/// msg may be a plain string or a stream expression: "value: " << x
#define LOG(level, msg)                                     \
    do                                                      \
    {                                                       \
        if (mapping::detail::shouldLog(                     \
                mapping::LogLevel::level, _logModuleLevel)) \
        {                                                   \
            std::stringstream sstr;                         \
            sstr << msg;                                    \
            mapping::detail::logImpl(                       \
                mapping::LogLevel::level, _logModule,       \
                std::nullopt, {sstr.str()});                \
        }                                                   \
    } while (0)

/// @brief Emit a single-line log message with a timestamp.
/// ts is a double in seconds; formatted as fixed-point with 4 decimal places.
#define LOG_STAMPED(level, ts, msg)                         \
    do                                                      \
    {                                                       \
        if (mapping::detail::shouldLog(                     \
                mapping::LogLevel::level, _logModuleLevel)) \
        {                                                   \
            std::stringstream sstr;                         \
            sstr << msg;                                    \
            mapping::detail::logImpl(                       \
                mapping::LogLevel::level, _logModule,       \
                (ts), {sstr.str()});                        \
        }                                                   \
    } while (0)

/// @brief Emit a multi-line log message.
/// The first argument after level is the header line; subsequent arguments are
/// continuation lines indented by four spaces. Plain string literals and STREAM()
/// expressions may be mixed freely.
#define LOG_MULTI(level, ...)                               \
    do                                                      \
    {                                                       \
        if (mapping::detail::shouldLog(                     \
                mapping::LogLevel::level, _logModuleLevel)) \
        {                                                   \
            mapping::detail::logImpl(                       \
                mapping::LogLevel::level, _logModule,       \
                std::nullopt, {__VA_ARGS__});               \
        }                                                   \
    } while (0)

/// @brief Emit a multi-line log message with a timestamp.
#define LOG_MULTI_STAMPED(level, ts, ...)                   \
    do                                                      \
    {                                                       \
        if (mapping::detail::shouldLog(                     \
                mapping::LogLevel::level, _logModuleLevel)) \
        {                                                   \
            mapping::detail::logImpl(                       \
                mapping::LogLevel::level, _logModule,       \
                (ts), {__VA_ARGS__});                       \
        }                                                   \
    } while (0)

/// @brief Wrap a stream expression as a std::string for use as a LOG_MULTI argument.
/// Use when an argument contains non-string values: STREAM("pose: " << p.translation())
#define STREAM(msg) ([&]() { std::stringstream _s; _s << msg; return _s.str(); }())

#endif // MAPPING_LOGGING_HPP_
