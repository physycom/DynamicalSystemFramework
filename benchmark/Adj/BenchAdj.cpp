#include <cstdint>

#include "RoadNetwork.hpp"
#include "Itinerary.hpp"
#include "FirstOrderDynamics.hpp"
#include "SparseMatrix.hpp"
#include "Bench.hpp"

using namespace dsm;

using Bench = sb::Bench<long long int>;

int main() {
  RoadNetwork graph{};
  graph.importOSMNodes("../test/data/forlì_nodes.csv");
  graph.importOSMEdges("../test/data/forlì_edges.csv");
  graph.buildAdj();
  auto const& adj{graph.adjMatrix()};
  auto const N{adj.n()};
  SparseMatrix<bool> sm(N, N);
  for (const auto& [srcId, dstId] : adj.elements()) {
    sm.insert(srcId, dstId);
  }


  const int n_rep{1000};
  Bench b1(n_rep), b2(n_rep), b3(n_rep), b4(n_rep);
  Logger::info("Benchmarking SparseMatrix::getCol");
  b1.benchmark([&sm, &N]() -> void { 
    for (size_t i{0}; i < N; ++i) {
      sm.getCol(i);
    }
  });
  b1.print<sb::microseconds>();
  Logger::info("Benchmarking SparseMatrix::getRow");
  b2.benchmark([&sm, &N]() -> void { 
    for (size_t i{0}; i < N; ++i) {
      sm.getRow(i);
    }
  });
  b2.print<sb::microseconds>();
  Logger::info("Benchmarking AdjacencyMatrix::getCol");
  b3.benchmark([&adj, &N]() -> void { 
    for (size_t i{0}; i < N; ++i) {
      adj.getCol(i);
    }
  });
  b3.print<sb::microseconds>();
  Logger::info("Benchmarking AdjacencyMatrix::getRow");
  b4.benchmark([&adj, &N]() -> void { 
    for (size_t i{0}; i < N; ++i) {
      adj.getRow(i);
    }
  });
  b4.print<sb::microseconds>();
}
