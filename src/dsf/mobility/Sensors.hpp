/// @file      /src/dsf/headers/Sensors.hpp
/// @brief     Defines some sensor classes.
///
/// @details   This file contains the definition of some sensor classes.
///            The Counter class is used to count events.

#pragma once

#include <utility>

#include "../utility/Typedef.hpp"

namespace dsf::mobility {
  /// @brief The Counter class is used to count events.
  class Counter {
  protected:
    std::string m_name{"N/A"};
    std::size_t m_counter{0};

  public:
    inline void setName(std::string const& name) { m_name = name; }
    inline Counter& operator++() {
      ++m_counter;
      return *this;
    }
    inline void reset() { m_counter = 0; }

    /// @brief Get the name of the counter
    /// @return The name of the counter
    inline auto const& name() const noexcept { return m_name; }
    /// @brief Get the value of the counter
    /// @return The value of the counter
    inline auto value() const noexcept { return m_counter; }
  };
}  // namespace dsf::mobility