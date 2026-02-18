/// @file       /src/dsf/mobility/MarkovianRoadNetwork.hpp
/// @brief      Defines the MarkovianRoadNetwork class.
///
/// @details    This file contains the definition of the MarkovianRoadNetwork class.
///             The MarkovianRoadNetwork class represents a simple road network
///             using MarkovianRoad edges and RoadJunction nodes. It does not support
///             special intersection types (traffic lights, roundabouts, etc.).

#pragma once

#include "../base/Network.hpp"
#include "RoadJunction.hpp"
#include "MarkovianRoad.hpp"
#include "../utility/Typedef.hpp"

#include <algorithm>
#include <format>
#include <iostream>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <string>

#include <spdlog/spdlog.h>

namespace dsf::mobility {
  /// @brief The MarkovianRoadNetwork class represents a simple road network
  ///        with MarkovianRoad edges and RoadJunction nodes.
  class MarkovianRoadNetwork : public Network<RoadJunction, MarkovianRoad> {
  private:
    unsigned long long m_capacity{0};

    void m_updateMaxAgentCapacity();

  public:
    MarkovianRoadNetwork() = default;
    // Disable copy
    MarkovianRoadNetwork(const MarkovianRoadNetwork&) = delete;
    MarkovianRoadNetwork& operator=(const MarkovianRoadNetwork&) = delete;
    // Enable move
    MarkovianRoadNetwork(MarkovianRoadNetwork&&) = default;
    MarkovianRoadNetwork& operator=(MarkovianRoadNetwork&&) = default;

    /// @brief Add a MarkovianRoad to the network
    /// @param road The MarkovianRoad to add
    void addRoad(MarkovianRoad&& road);

    /// @brief Set the stationary weights for each road
    /// @param roadWeights A map where the key is the road id and the value is the weight.
    ///        Roads not present in the map keep their default weight of 1.0.
    void setRoadStationaryWeights(std::unordered_map<Id, double> const& roadWeights);

    /// @brief Get the maximum agent capacity of the network
    /// @return unsigned long long The maximum agent capacity
    inline auto capacity() const noexcept { return m_capacity; }

    /// @brief Describe the MarkovianRoadNetwork
    /// @param os The output stream (default is std::cout)
    void describe(std::ostream& os = std::cout) const;
  };

  inline void MarkovianRoadNetwork::m_updateMaxAgentCapacity() {
    m_capacity = 0;
    for (auto const& [_, pRoad] : this->edges()) {
      m_capacity += pRoad->capacity();
    }
  }

  inline void MarkovianRoadNetwork::addRoad(MarkovianRoad&& road) {
    m_capacity += road.capacity();
    auto const& nodes{this->nodes()};
    if (!nodes.contains(road.source())) {
      spdlog::debug("Node with id {} not found, adding default", road.source());
      addNode(road.source());
    }
    if (!nodes.contains(road.target())) {
      spdlog::debug("Node with id {} not found, adding default", road.target());
      addNode(road.target());
    }
    addEdge<MarkovianRoad>(std::move(road));
  }

  inline void MarkovianRoadNetwork::setRoadStationaryWeights(
      std::unordered_map<Id, double> const& roadWeights) {
    for (auto& [roadId, pRoad] : this->m_edges) {
      if (roadWeights.contains(roadId)) {
        pRoad->setStationaryWeight(roadWeights.at(roadId));
      } else {
        pRoad->setStationaryWeight(1.0);
      }
    }
  }

  inline void MarkovianRoadNetwork::describe(std::ostream& os) const {
    os << "MarkovianRoadNetwork:\n"
       << "  Nodes: " << nNodes() << '\n'
       << "  Edges: " << nEdges() << '\n'
       << "  Capacity: " << m_capacity << '\n';
  }
}  // namespace dsf::mobility

