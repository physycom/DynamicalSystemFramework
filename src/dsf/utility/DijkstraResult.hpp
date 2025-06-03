/// @file utility/DijkstraResult.hpp
/// @brief This file contains the definition of the DijkstraResult class.
///
/// @details The DijkstraResult class represents the result of a Dijkstra algorithm.
/// 		 It is templated by the type of the graph's id.

#pragma once

#include "Typedef.hpp"

namespace dsf {

  /// @brief The DijkstraResult class represents the result of a Dijkstra algorithm.
  /// @tparam Id, The type of the graph's id. Must be an unsigned integral type.
  class DijkstraResult {
  private:
    std::vector<Id> m_path;
    double m_distance;

  public:
    DijkstraResult() : m_path{}, m_distance{-1.} {}
    /// @brief Construct a new DijkstraResult object
    /// @param path, A vector of the ids of the nodes in the path
    /// @param distance, The distance of the path
    /// @details The path is represented by a vector of the ids of the nodes in the path.
    DijkstraResult(std::vector<Id> path, double distance)
        : m_path{std::move(path)}, m_distance{distance} {}

    /// @brief Get the path
    /// @return A vector of the ids of the nodes in the path
    /// @details The path is represented by a vector of the ids of the nodes in the path.
    const std::vector<Id>& path() const { return m_path; }
    /// @brief Get the distance
    /// @return The distance of the path
    double distance() const { return m_distance; }
  };
};  // namespace dsf
