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
  // Take 10 random keys from nodes map
  {
    std::vector<dsf::Id> nodeIds;
    nodeIds.reserve(dynamics.graph().nNodes());
    for (const auto& pair : dynamics.graph().nodes()) {
      nodeIds.push_back(pair.first);
    }
    std::vector<dsf::Id> randomNodeIds;
    std::sample(nodeIds.begin(),
                nodeIds.end(),
                std::back_inserter(randomNodeIds),
                10,
                std::mt19937{std::random_device{}()});
    dynamics.setDestinationNodes(randomNodeIds, false);
  }

  const int n_rep{1};
  Bench b1(n_rep);
  std::cout << "Benchmarking updatePaths\n";
  b1.benchmark([&dynamics]() -> void { dynamics.updatePaths(); });
  std::cout << "Time elapsed (s):\n";
  b1.print<sb::seconds>();
}
