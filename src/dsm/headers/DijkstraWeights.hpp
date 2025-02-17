#pragma once

#include "../utility/Typedef.hpp"

namespace dsm {

  class Graph;

  /// @brief Returns the length of a street given its source and destination nodes
  /// @param graph A pointer to the graph
  /// @param node1 The source node id
  /// @param node2 The destination node id
  /// @return The length of the street
  double streetLength(const Graph* graph, Id node1, Id node2);
  /// @brief Returns the minimum time to cross a street given its source and destination nodes
  /// @param graph A pointer to the graph
  /// @param node1 The source node id
  /// @param node2 The destination node id
  /// @return The minimum time to cross the street
  /// @details The minimum time is the ratio between the street's length and the maximum speed
  double streetMinTime(const Graph* graph, Id node1, Id node2);
  /// @brief Returns the time to cross a street given its source and destination nodes
  /// @param graph A pointer to the graph
  /// @param node1 The source node id
  /// @param node2 The destination node id
  /// @return The time to cross the street
  /// @details This time also takes into account the number of agents on the street
  double streetTime(const Graph* graph, Id node1, Id node2);

};  // namespace dsm
