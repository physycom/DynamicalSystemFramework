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
  auto const N{graph.nNodes()};
  SparseMatrix<bool> sm;
  for (const auto& [_, pEdge] : graph.edges()) {
    sm.insert(pEdge->source(), pEdge->target(), true);
  }

  const int n_rep{100};
  Bench b1(n_rep), b2(n_rep), b3(n_rep), b4(n_rep), b5(n_rep);
  Logger::info("Benchmarking SparseMatrix::getCol");
  b1.benchmark([&sm, &N]() -> void {
    for (size_t i{0}; i < N; ++i) {
      sm.col(i);
    }
  });
  b1.print<sb::microseconds>();
  Logger::info("Benchmarking SparseMatrix::getRow");
  b2.benchmark([&sm, &N]() -> void {
    for (size_t i{0}; i < N; ++i) {
      sm.row(i);
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
  Logger::info("Benchmarking AdjacencyMatrix::getRow");
  b5.benchmark([&graph]() -> void {
    for (auto const& pair : graph.nodes()) {
      auto const& pNodeCopy{graph.node(pair.first)};
    }
  });
  b5.print<sb::microseconds>();
}
