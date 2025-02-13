/// @file       /src/dsm/headers/Itinerary.hpp
/// @brief      Defines the Itinerary class.
///
/// @details    This file contains the definition of the Itinerary class.
///             The Itinerary class represents an itinerary in the network. It is templated
///             by the type of the itinerary's id, which must be an unsigned integral type.
///             An itinerary is defined by its id, its destination and the path to reach it.

#pragma once

#include "AdjacencyMatrix.hpp"
#include "../utility/Typedef.hpp"

#include <concepts>
#include <utility>
#include <string>
#include <format>
#include <memory>

namespace dsm {
  class AdjacencyMatrix;
  /// @brief The Itinerary class represents an itinerary in the network.
  /// @tparam Id The type of the itinerary's id. It must be an unsigned integral type.
  class Itinerary {
  private:
    Id m_id;
    Id m_destination;
    std::unique_ptr<AdjacencyMatrix> m_path;

  public:
    /// @brief Construct a new Itinerary object
    /// @param destination The itinerary's destination
    Itinerary(Id id, Id destination);

    // Allow move constructor and move assignment operator
    Itinerary(Itinerary&&) = default;
    Itinerary& operator=(Itinerary&&) = default;
    // Delete copy constructor and copy assignment operator
    Itinerary(const Itinerary&) = delete;
    Itinerary& operator=(const Itinerary&) = delete;

    /// @brief Set the itinerary's path
    /// @param path An adjacency matrix made by a SparseMatrix representing the itinerary's path
    /// @throw std::invalid_argument, if the itinerary's source or destination is not in the path's
    void setPath(AdjacencyMatrix path);

    /// @brief Get the itinerary's id
    /// @return Id, The itinerary's id
    Id id() const;
    /// @brief Get the itinerary's destination
    /// @return Id, The itinerary's destination
    Id destination() const;
    /// @brief Get the itinerary's path
    /// @return AdjacencyMatrix An adjacency matrix representing the itinerary's path
    std::unique_ptr<AdjacencyMatrix> const& path() const;
  };
};  // namespace dsm
