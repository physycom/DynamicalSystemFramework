#include <cstdint>

#include "Graph.hpp"
#include "Itinerary.hpp"
#include "FirstOrderDynamics.hpp"
#include "Bench.hpp"

using Graph = dsm::Graph;
using Itinerary = dsm::Itinerary;
using Dynamics = dsm::FirstOrderDynamics;

using Bench = sb::Bench<long long int>;

int main() {
  Graph graph{};
  graph.importMatrix("../test/data/matrix.dat", false);
  for (const auto& [streetId, street] : graph.streetSet()) {
    street->setMaxSpeed(13.9);
  }

  Dynamics dynamics{graph};
  dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(0, 118)));
  dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(4, 115)));
  dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(8, 112)));
  dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(12, 109)));

  const int n_rep{100};
  Bench b1(n_rep);
  std::cout << "Benchmarking updatePaths\n";
  dynamics.updatePaths();
  b1.benchmark([&dynamics]() -> void { dynamics.updatePaths(); });
  b1.print<sb::milliseconds>();
}
