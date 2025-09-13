#include <cstdint>

#include "RoadNetwork.hpp"
#include "Itinerary.hpp"
#include "FirstOrderDynamics.hpp"
#include "SparseMatrix.hpp"
#include "Bench.hpp"

using namespace dsf;

using Bench = sb::Bench<long long int>;

int main() {
  RoadNetwork graph{};
  graph.importOSMNodes("../test/data/postua_nodes.csv");
  graph.importOSMEdges("../test/data/postua_edges.csv");
  SparseMatrix<bool> sm;
  for (const auto& [_, pEdge] : graph.edges()) {
    sm.insert(pEdge->source(), pEdge->target(), true);
  }
  std::vector<Id> nodeIndices;
  nodeIndices.reserve(graph.nNodes());
  for (const auto& [nodeId, _] : graph.nodes()) {
    nodeIndices.push_back(nodeId);
  }

  const int n_rep{100};
  Bench b1(n_rep), b2(n_rep), b3(n_rep), b4(n_rep);
  Logger::info("Benchmarking SparseMatrix::getCol");
  b1.benchmark([&sm, &nodeIndices]() -> void {
    for (auto const& nodeId : nodeIndices) {
      sm.col(nodeId);
    }
  });
  b1.print<sb::microseconds>();
  Logger::info("Benchmarking SparseMatrix::getRow");
  b2.benchmark([&sm, &nodeIndices]() -> void {
    for (auto const& nodeId : nodeIndices) {
      sm.row(nodeId);
    }
  });
  b2.print<sb::microseconds>();
  Logger::info("Benchmarking AdjacencyMatrix::getCol");
  b3.benchmark([&graph]() -> void {
    for (auto const& pair : graph.nodes()) {
      pair.second->ingoingEdges();
    }
  });
  b3.print<sb::microseconds>();
  Logger::info("Benchmarking AdjacencyMatrix::getRow");
  b4.benchmark([&graph]() -> void {
    for (auto const& pair : graph.nodes()) {
      pair.second->outgoingEdges();
    }
  });
  b4.print<sb::microseconds>();
}
