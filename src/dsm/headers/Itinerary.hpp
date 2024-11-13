/// @file       /src/dsm/headers/Itinerary.hpp
/// @brief      Defines the Itinerary class.
///
/// @details    This file contains the definition of the Itinerary class.
///             The Itinerary class represents an itinerary in the network. It is templated
///             by the type of the itinerary's id, which must be an unsigned integral type.
///             An itinerary is defined by its id, its destination and the path to reach it.

#pragma once

#include "SparseMatrix.hpp"
#include "../utility/Typedef.hpp"

#include <concepts>
#include <utility>
#include <string>
#include <format>

namespace dsm {
  /// @brief The Itinerary class represents an itinerary in the network.
  /// @tparam Id The type of the itinerary's id. It must be an unsigned integral type.
  class Itinerary {
  private:
    SparseMatrix<bool> m_path;
    Id m_destination;

  public:
    /// @brief Construct a new Itinerary object
    /// @param destination The itinerary's destination
    Itinerary(Id destination);
    /// @brief Construct a new Itinerary object
    /// @param destination The itinerary's destination
    /// @param path An adjacency matrix made by a SparseMatrix representing the itinerary's path
    Itinerary(Id destination, SparseMatrix<bool> path);

    /// @brief Set the itinerary's path
    /// @param path An adjacency matrix made by a SparseMatrix representing the itinerary's path
    /// @throw std::invalid_argument, if the itinerary's source or destination is not in the path's
    void setPath(SparseMatrix<bool> path);

    /// @brief Get the itinerary's destination
    /// @return Id, The itinerary's destination
    Id destination() const { return m_destination; }
    /// @brief Get the itinerary's path
    /// @return SparseMatrix<Id, bool>, An adjacency matrix made by a SparseMatrix representing the
    /// itinerary's path
    const SparseMatrix<bool>& path() const { return m_path; }
  };
};  // namespace dsm
