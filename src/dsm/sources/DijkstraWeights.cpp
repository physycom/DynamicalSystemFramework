
#include "../headers/DijkstraWeights.hpp"
#include "../headers/Graph.hpp"

namespace dsm {

  double streetLength(const Graph* graph, Id node1, Id node2) {
    const auto street{graph->street(node1, node2)};
    return (*street)->length();
  }

  double streetTime(const Graph* graph, Id node1, Id node2) {
    const auto street{graph->street(node1, node2)};
    const auto length{(*street)->length()};
    const auto speed{(*street)->maxSpeed() *
                     (1. - (*street)->nAgents() / (*street)->capacity())};

    return length / speed;
  }

};  // namespace dsm
