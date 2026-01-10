/// @file       /src/dsf/headers/Itinerary.hpp
/// @brief      Defines the Itinerary class.
///
/// @details    This file contains the definition of the Itinerary class.
///             The Itinerary class represents an itinerary in the network. It is templated
///             by the type of the itinerary's id, which must be an unsigned integral type.
///             An itinerary is defined by its id, its destination and the path to reach it.

#pragma once

#include "PathCollection.hpp"
#include "../utility/Typedef.hpp"

#include <concepts>
#include <utility>
#include <string>
#include <format>
#include <memory>
#include <map>
#include <vector>

namespace dsf::mobility {
  /// @brief The Itinerary class represents an itinerary in the network.
  class Itinerary {
  private:
    Id m_source, m_destination;
    std::multimap<double, std::shared_ptr<std::vector<Id>>> m_paths; // key: path cost, value: path (vector of street IDs)

  public:
    /// @brief Construct a new Itinerary object
    /// @param source The itinerary's source
    /// @param destination The itinerary's destination
    Itinerary(Id const& source, Id const& destination);

    /// @brief Set the itinerary's paths
    /// @param paths The itinerary's paths
    void setPaths(std::multimap<double, std::vector<Id>> const& paths);

    /// @brief Get the itinerary's source
    /// @return Id, The itinerary's source
    inline auto source() const noexcept { return m_source; };
    /// @brief Get the itinerary's destination
    /// @return Id, The itinerary's destination
    inline auto destination() const noexcept { return m_destination; };
    /// @brief Get the itinerary's path
    /// @return PathCollection const&, The itinerary's path
    inline auto const& paths() const noexcept { return m_paths; };
  };
};  // namespace dsf::mobility
