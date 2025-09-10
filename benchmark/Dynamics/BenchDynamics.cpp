#include <cstdint>

#include "RoadNetwork.hpp"
#include "Itinerary.hpp"
#include "FirstOrderDynamics.hpp"
#include "Bench.hpp"

using RoadNetwork = dsf::RoadNetwork;
using Itinerary = dsf::Itinerary;
using Dynamics = dsf::FirstOrderDynamics;

using Bench = sb::Bench<long long int>;

int main() {
  RoadNetwork graph{};
  graph.importOSMNodes("../test/data/forlì_nodes.csv");
  graph.importOSMEdges("../test/data/forlì_edges.csv");

  Dynamics dynamics{graph};
  dynamics.setDestinationNodes({10, 42, 69, 121, 420, 690, 777, 999, 1020, 1212}, false);

  const int n_rep{1};
  Bench b1(n_rep);
  std::cout << "Benchmarking updatePaths\n";
  b1.benchmark([&dynamics]() -> void { dynamics.updatePaths(); });
  std::cout << "Time elapsed (s):\n";
  b1.print<sb::seconds>();
}
