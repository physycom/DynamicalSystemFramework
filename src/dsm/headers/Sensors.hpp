/// @file      /src/dsm/headers/Sensors.hpp
/// @brief     Defines some sensor classes.
///
/// @details   This file contains the definition of some sensor classes.
///            The Counter class contains two counters to count input and output.

#pragma once

#include <string>
#include <utility>

namespace dsm {
  /// @brief The Counter class contains two counters to count input and output.
  class Counter {
  protected:
    std::string m_code;
    std::pair<int, int> m_counters = {0, 0};  // First = in, Second = out
  public:
    void setCode(std::string const& code);
    /// @brief Increase the input counter by one
    void increaseInputCounter();
    /// @brief Increase the output counter by one
    void increaseOutputCounter();

    std::string const& code() const;
    /// @brief Get the number of input counts registered
    /// @param reset If true, the counter is reset to 0. Default is true
    int inputCounts(bool reset = true);
    /// @brief Get the number of output counts registered
    /// @param reset If true, the counter is reset to 0. Default is true
    int outputCounts(bool reset = true);
  };
}  // namespace dsm