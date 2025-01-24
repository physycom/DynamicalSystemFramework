
#pragma once

#include <cstdlib>
#include <format>
#include <iostream>
#include <source_location>
#include <string>

static std::string buildMessage(const std::string& type,
                                const std::string& message,
                                const std::source_location& location) {
  return std::format("{}({}:{}) \'{}\': {}",
                     type,
                     location.file_name(),
                     location.line(),
                     location.function_name(),
                     message);
}

namespace dsm {

  /// @brief The Logger class is a simple logging class.
  struct Logger {
    inline std::string buildExceptionMessage(
        const std::string& message,
        const std::source_location& location = std::source_location::current()) {
      return buildMessage(std::string(), message, location);
    };
    inline void info(
        const std::string& message,
        const std::source_location& location = std::source_location::current()) {
      std::clog << buildMessage("\033[38;2;0;0;255mINFO ", message, location) +
                       "\033[0m\n";
    };
    inline void debug(
        const std::string& message,
        const std::source_location& location = std::source_location::current()) {
#ifndef NDEBUG
      std::clog << buildMessage("\033[38;2;0;255;0mDEBUG ", message, location) +
                       "\033[0m\n";
#endif
    };
    inline void warning(
        const std::string& message,
        const std::source_location& location = std::source_location::current()) {
      std::clog << buildMessage("\033[38;2;130;30;180mWARNING ", message, location) +
                       "\033[0m\n";
    };
    inline void error(
        const std::string& message,
        const std::source_location& location = std::source_location::current()) {
      std::cerr << buildMessage("\033[38;5;196mERROR ", message, location) + "\033[0m\n";
      std::abort();
    }
  };

  static Logger logger;
}  // namespace dsm
