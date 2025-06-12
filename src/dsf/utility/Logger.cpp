#include "Logger.hpp"

namespace dsf {
  bool Logger::m_verbose{false};
  log_level_t Logger::m_logLevel{log_level_t::INFO};

  void Logger::setVerbose(bool verbose) { m_verbose = verbose; };
  void Logger::setLogLevel(log_level_t logLevel) { m_logLevel = logLevel; };

  std::string Logger::buildExceptionMessage(const std::string& message,
                                            const std::source_location& location) {
    return buildMessage(std::string(), message, location, m_verbose);
  };

  void Logger::debug(const std::string& message, const std::source_location& location) {
    if (m_logLevel > log_level_t::DEBUG) {
      return;
    }
    std::clog << buildMessage("\033[38;2;255;255;0mDEBUG", message, location, m_verbose) +
                     "\033[0m\n";
  };
  void Logger::info(const std::string& message, const std::source_location& location) {
    if (m_logLevel > log_level_t::INFO) {
      return;
    }
    std::clog << buildMessage("\033[1;32mINFO", message, location, m_verbose) +
                     "\033[1;0m\n";
  };
  void Logger::warning(const std::string& message, const std::source_location& location) {
    if (m_logLevel > log_level_t::WARNING) {
      return;
    }
    std::clog << buildMessage(
                     "\033[38;2;130;30;180mWARNING", message, location, m_verbose) +
                     "\033[1;0m\n";
  };

}  // namespace dsf