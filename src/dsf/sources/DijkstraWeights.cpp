
#include "../headers/DijkstraWeights.hpp"
#include "../headers/RoadNetwork.hpp"

namespace dsf {

  namespace weight_functions {
    double streetLength(const RoadNetwork* graph, Id node1, Id node2) {
      const auto street{graph->street(node1, node2)};
      return (*street)->length();
    }

    double streetTime(const RoadNetwork* graph, Id node1, Id node2) {
      const auto street{graph->street(node1, node2)};
      const auto length{(*street)->length()};
      const auto speed{(*street)->maxSpeed() *
                       (1. - (*street)->nAgents() / (*street)->capacity())};
      return std::ceil(length / speed);
    }
  }  // namespace weight_functions

};  // namespace dsf
