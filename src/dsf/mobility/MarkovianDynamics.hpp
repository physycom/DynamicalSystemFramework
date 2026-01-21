#pragma once

#include "AgentDynamics.hpp"
#include "RoadNetwork.hpp"

namespace dsf::mobility {
  class MarkovianDynamics : public AgentDynamics<RoadNetwork> {
    public:
        MarkovianDynamics(RoadNetwork& graph,
                        std::optional<unsigned int> seed = std::nullopt)
            : AgentDynamics<RoadNetwork>(graph, seed) {}
  };
} // namespace dsf::mobility