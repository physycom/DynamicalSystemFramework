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
  graph.importOSMNodes("../test/data/forlì_nodes.csv");
  graph.importOSMEdges("../test/data/forlì_edges.csv");
  graph.buildAdj();

  Dynamics dynamics{graph};
  std::vector<dsm::Id> destinations{10, 42, 69, 121, 420, 690, 777, 999, 1020, 1212};
  dynamics.setDestinationNodes(destinations);

  const int n_rep{1};
  Bench b1(n_rep);
  std::cout << "Benchmarking updatePaths\n";
  dynamics.updatePaths();
  b1.benchmark([&dynamics]() -> void { dynamics.updatePaths(); });
  std::cout << "Time elapsed (ms):\n";
  b1.print<sb::milliseconds>();
}
