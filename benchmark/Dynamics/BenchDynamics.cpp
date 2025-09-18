#include <cstdint>

#include "RoadNetwork.hpp"
#include "Itinerary.hpp"
#include "FirstOrderDynamics.hpp"
#include "Bench.hpp"

#include <spdlog/spdlog.h>

using RoadNetwork = dsf::RoadNetwork;
using Itinerary = dsf::Itinerary;
using Dynamics = dsf::FirstOrderDynamics;

using Bench = sb::Bench<long long int>;

int main() {
  // Declare generator
  std::mt19937_64 generator{std::random_device{}()};
  generator.seed(69);
  RoadNetwork graph{};
  graph.importNodes("../test/data/forlì_nodes.csv");
  graph.importEdges("../test/data/forlì_edges.csv");

  Dynamics dynamics{graph};
  // Take 10 random keys from nodes map
  std::vector<dsf::Id> randomNodeIds;
  {
    std::vector<dsf::Id> nodeIds;
    nodeIds.reserve(dynamics.graph().nNodes());
    for (const auto& pair : dynamics.graph().nodes()) {
      nodeIds.push_back(pair.first);
    }
    std::sample(
        nodeIds.begin(), nodeIds.end(), std::back_inserter(randomNodeIds), 10, generator);
  }
  dynamics.setDestinationNodes(randomNodeIds);

  const int n_rep{100};
  Bench b1(n_rep);
  std::cout << "Benchmarking updatePaths\n";
  b1.benchmark([&dynamics]() -> void { dynamics.updatePaths(); });
  std::cout << "Time elapsed after " << n_rep << " repetitions (us):\n";
  b1.print<sb::microseconds>();

  for (auto const& [itineraryId, pItinerary] : dynamics.itineraries()) {
    auto const& path = pItinerary->path();
    auto const& size = path.size();
    double avgPossibleMoves{0.};
    for (auto const& [nodeId, nextHops] : path) {
      avgPossibleMoves += nextHops.size();
    }
    double avgDegree{0.};
    for (auto const& nodeId : dynamics.graph().nodes()) {
      avgDegree += nodeId.second->outgoingEdges().size();
    }
    avgDegree /= dynamics.graph().nNodes();
    avgPossibleMoves /= size;
    spdlog::info("Itinerary {}: {} nodes, avg possible moves: {:.2f}, avg degree: {:.2f}",
                 itineraryId,
                 size,
                 avgPossibleMoves,
                 avgDegree);
  }
}
