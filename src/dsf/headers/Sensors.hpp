/// @file      /src/dsf/headers/Sensors.hpp
/// @brief     Defines some sensor classes.
///
/// @details   This file contains the definition of some sensor classes.
///            The Counter class contains two counters to count input and output.

#pragma once

#include <utility>

#include "../utility/Typedef.hpp"

namespace dsf {
  /// @brief The Counter class contains two counters to count input and output.
  class Counter {
  protected:
    Id m_code;
    std::pair<int, int> m_counters = {0, 0};  // First = in, Second = out
  public:
    void setCode(Id const code);
    /// @brief Increase the input counter by one
    void increaseInputCounter();
    /// @brief Increase the output counter by one
    void increaseOutputCounter();

    Id code() const;
    /// @brief Get the number of input counts registered
    /// @param reset If true, the counter is reset to 0. Default is true
    int inputCounts(bool reset = true);
    /// @brief Get the number of output counts registered
    /// @param reset If true, the counter is reset to 0. Default is true
    int outputCounts(bool reset = true);
  };
}  // namespace dsf