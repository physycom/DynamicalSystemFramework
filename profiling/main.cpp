#include "dsf/dsf.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using unit = uint32_t;

using RoadNetwork = dsf::RoadNetwork;
using Itinerary = dsf::Itinerary;
using Dynamics = dsf::FirstOrderDynamics;

int main() {
  RoadNetwork graph{};
  std::cout << "Importing matrix.dat...\n";
  graph.importMatrix("../test/data/rawMatrix.dat", false);
  std::cout << "Number of nodes: " << graph.nodes().size() << '\n'
            << "Number of streets: " << graph.edges().size() << '\n';
  for (auto& streetPair : graph.edges()) {
    auto& street = streetPair.second;
    street->setCapacity(100);
    street->setMaxSpeed(10.);
  }
  for (auto& nodePair : graph.nodes()) {
    auto& node = nodePair.second;
    node->setCapacity(10);
  }
  std::cout << "Done.\n";

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph, false, std::nullopt, 0.95};
  dynamics.addItinerary(0, 118);
  dynamics.addItinerary(1, 115);
  dynamics.addItinerary(2, 112);
  dynamics.addItinerary(3, 109);
  dynamics.setErrorProbability(0.3);
  dynamics.updatePaths();

  std::cout << "Done.\n"
            << "Running simulation...\n";

  for (unsigned int i{0}; i < 1000; ++i) {
    if (i < 12e3) {
      if (i % 60 == 0) {
        dynamics.addAgentsUniformly(100);
      }
    }
    dynamics.evolve(false);
  }
  std::cout << '\n' << "Done.\n";

  return 0;
}
