#pragma once

#include "../utility/Typedef.hpp"

namespace dsf {

  class RoadNetwork;

  namespace weight_functions {
    /// @brief Returns the length of a street given its source and destination nodes
    /// @param graph A pointer to the graph
    /// @param edgeId The edge id
    /// @return The length of the street
    double streetLength(const RoadNetwork* graph,
                        Id edgeId,
                        [[maybe_unused]] double param);
    /// @brief Returns the time to cross a street given its source and destination nodes
    /// @param graph A pointer to the graph
    /// @param edgeId The edge id
    /// @param param A parameter to adjust the influence of the density on the speed (default is 0)
    /// @return The time to cross the street
    /// @details This time also takes into account the number of agents on the street
    double streetTime(const RoadNetwork* graph, Id edgeId, [[maybe_unused]] double param);
    /// @brief Returns the weight of a street given its source and destination nodes
    /// @param graph A pointer to the graph
    /// @param edgeId The edge id
    /// @return The weight of the street
    /// @details The weight is a custom weight for the street, which can be used to
    /// influence the routing algorithm. If no custom weight is set, it throws an exception.
    double streetWeight(const RoadNetwork* graph,
                        Id edgeId,
                        [[maybe_unused]] double param);
  }  // namespace weight_functions

};  // namespace dsf
