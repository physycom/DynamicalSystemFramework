#pragma once

#include "../utility/Typedef.hpp"

namespace dsm {

  class RoadNetwork;

  namespace weight_functions {
    /// @brief Returns the length of a street given its source and destination nodes
    /// @param graph A pointer to the graph
    /// @param node1 The source node id
    /// @param node2 The destination node id
    /// @return The length of the street
    double streetLength(const RoadNetwork* graph, Id node1, Id node2);
    /// @brief Returns the time to cross a street given its source and destination nodes
    /// @param graph A pointer to the graph
    /// @param node1 The source node id
    /// @param node2 The destination node id
    /// @return The time to cross the street
    /// @details This time also takes into account the number of agents on the street
    double streetTime(const RoadNetwork* graph, Id node1, Id node2);
  }  // namespace weight_functions

};  // namespace dsm
