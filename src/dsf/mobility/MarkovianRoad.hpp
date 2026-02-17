#pragma once

#include "Agent.hpp"
#include "Road.hpp"

#include "../utility/queue.hpp"

#include <format>
#include <memory>

namespace dsf::mobility {
  class MarkovianRoad : public Road {
  private:
    dsf::queue<std::unique_ptr<Agent>> m_queue;
    double m_stationaryWeight{1.0};

  public:
    // Use road constructors
    using Road::Road;

    void enqueue(std::unique_ptr<Agent> pAgent);

    std::unique_ptr<Agent> dequeue();

    inline std::size_t nAgents() const final { return m_queue.size(); }
  };

}  // namespace dsf::mobility

template <>
struct std::formatter<dsf::mobility::MarkovianRoad> : std::formatter<std::string> {
  auto format(dsf::mobility::MarkovianRoad const& road, auto& ctx) {
    return std::format_to(ctx.out(),
                          "MarkovianRoad(id={}, nodePair=({}, {}), length={}, "
                          "maxSpeed={}, nLanes={}, name=\"{}\", nAgents={})",
                          road.id(),
                          road.source(),
                          road.target(),
                          road.length(),
                          road.maxSpeed(),
                          road.nLanes(),
                          road.name(),
                          road.nAgents());
  }
};