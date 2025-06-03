#pragma once

#include "Typedef.hpp"

#include <format>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
#include <set>

static std::string buildMessage(const std::string& type,
                                const std::string& message,
                                const std::source_location& location,
                                bool verbose) {
  if (verbose) {
    return std::format("{} ({}:{}) \'{}\': {}",
                       type,
                       location.file_name(),
                       location.line(),
                       location.function_name(),
                       message);
  }
  return std::format("{}: {}", type, message);
}

namespace dsf {

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
    /// @brief Log an error message and throw an exception of the given type with the message
    /// @tparam ExceptionType The type of exception to throw
    /// @param message The message
    /// @param location The location of the message. Default is the current location
    template <typename ExceptionType = std::runtime_error>
    static void error(
        const std::string& message,
        const std::source_location& location = std::source_location::current());
  };
}  // namespace dsf

// Template implementation must be in the header
namespace dsf {
  template <typename ExceptionType>
  inline void Logger::error(const std::string& message,
                            const std::source_location& location) {
    std::string fullMessage =
        buildMessage("\033[1;31mERROR", message, location, m_verbose) + "\033[1;0m\n";
    std::cerr << fullMessage;
    throw ExceptionType(fullMessage);
  }
}  // namespace dsf
