#include "dsm/dsm.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using unit = uint32_t;

using Graph = dsm::Graph;
using Itinerary = dsm::Itinerary;
using Dynamics = dsm::FirstOrderDynamics;

int main() {
  Graph graph{};
  std::cout << "Importing matrix.dat...\n";
  graph.importMatrix("../test/data/rawMatrix.dat", false);
  std::cout << "Number of nodes: " << graph.nodeSet().size() << '\n'
            << "Number of streets: " << graph.streetSet().size() << '\n';
  for (auto& streetPair : graph.streetSet()) {
    auto& street = streetPair.second;
    street->setCapacity(100);
    street->setMaxSpeed(10.);
  }
  for (auto& nodePair : graph.nodeSet()) {
    auto& node = nodePair.second;
    node->setCapacity(10);
  }
  std::cout << "Done.\n";

  Itinerary it1{0, 118};
  Itinerary it2{1, 115};
  Itinerary it3{2, 112};
  Itinerary it4{3, 109};

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph, false, std::nullopt, 0.95};
  dynamics.addItinerary(it1);
  dynamics.addItinerary(it2);
  dynamics.addItinerary(it3);
  dynamics.addItinerary(it4);
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
