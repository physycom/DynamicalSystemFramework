#include "Logger.hpp"

#include <cstdlib>
#include <format>
#include <iostream>

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

namespace dsm {
  bool Logger::m_verbose{false};

  void Logger::setVerbose(bool verbose) { m_verbose = verbose; };

  std::string Logger::buildExceptionMessage(const std::string& message,
                                            const std::source_location& location) {
    return buildMessage(std::string(), message, location, m_verbose);
  };

  void Logger::debug(const std::string& message, const std::source_location& location) {
#ifndef NDEBUG
    std::clog << buildMessage("\033[38;2;0;255;0mDEBUG", message, location, m_verbose) +
                     "\033[0m\n";
#endif
  };
  void Logger::info(const std::string& message, const std::source_location& location) {
    std::clog << buildMessage("\033[1;32mINFO", message, location, m_verbose) +
                     "\033[1;0m\n";
  };
  void Logger::warning(const std::string& message, const std::source_location& location) {
    std::clog << buildMessage(
                     "\033[38;2;130;30;180mWARNING", message, location, m_verbose) +
                     "\033[1;0m\n";
  };
  void Logger::error(const std::string& message, const std::source_location& location) {
    std::cerr << buildMessage("\033[1;31mERROR", message, location, m_verbose) +
                     "\033[1;0m\n";
    std::abort();
  }

}  // namespace dsm