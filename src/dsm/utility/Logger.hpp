#pragma once

#include "Typedef.hpp"

#include <source_location>
#include <string>
#include <set>

namespace dsm {

  /// @brief The Logger class is a simple logging class which provides static methods to log messages on std::clog and std::cerr.
  class Logger {
    static log_level_t m_logLevel;
    static bool m_verbose;

  public:
    /// @brief Set the verbosity of the logger
    /// @param verbose If true, the logger will print the detailed location of the message
    static void setVerbose(bool verbose);
    /// @brief Set the log level of the logger
    /// @param logLevel The log level
    /// @details The log level can be DEBUG (0), INFO (1), WARNING(2) or ERROR (3)
    static void setLogLevel(log_level_t logLevel);
    /// @brief Build an exception message
    /// @param message The message
    /// @param location The location of the exception. Default is the current location
    /// @return std::string The exception message
    static std::string buildExceptionMessage(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
    /// @brief Log a debug message on std::clog, if NDEBUG is not defined
    /// @param message The message
    /// @param location The location of the message. Default is the current location
    static void debug(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
    /// @brief Log an info message on std::clog
    /// @param message The message
    /// @param location The location of the message. Default is the current location
    static void info(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
    /// @brief Log a warning message on std::clog
    /// @param message The message
    /// @param location The location of the message. Default is the current location
    static void warning(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
    /// @brief Log an error message on std::cerr and abort the program
    /// @param message The message
    /// @param location The location of the message. Default is the current location
    static void error(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
  };
}  // namespace dsm
