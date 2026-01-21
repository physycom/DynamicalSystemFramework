#pragma once

#include "Road.hpp"

namespace dsf::mobility {
  class MarkovianRoad : public Road {
  public:
    /// @brief Construct a new Markovian Road object
    /// @param id The road's id
    /// @param nodePair The road's node pair
    /// @param length The road's length, in meters (default is the mean vehicle length)
    /// @param nLanes The road's number of lanes (default is 1)
    /// @param maxSpeed The road's speed limit, in m/s (default is 50 km/h)
    /// @param name The road's name (default is an empty string)
    /// @param geometry The road's geometry (default is empty)
    /// @param capacity The road's capacity (default is the maximum number of vehicles that can fit in the road)
    /// @param transportCapacity The road's transport capacity (default is 1)
    MarkovianRoad(Id id,
                  std::pair<Id, Id> nodePair,
                  double length = Road::meanVehicleLength(),
                  double maxSpeed = 13.8888888889,
                  int nLanes = 1,
                  std::string name = std::string(),
                  geometry::PolyLine geometry = {},
                  std::optional<int> capacity = std::nullopt,
                  double transportCapacity = 1.)
        : Road(id,
               std::move(nodePair),
               length,
               maxSpeed,
               nLanes,
               std::move(name),
               std::move(geometry),
               capacity,
               transportCapacity) {}
    MarkovianRoad(MarkovianRoad&&) = default;
    MarkovianRoad(MarkovianRoad const&) = delete;
  };
}  // namespace dsf::mobility