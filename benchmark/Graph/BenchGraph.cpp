
#include <cstdint>
#include <iostream>
#include <random>
#include "Bench.hpp"

#include "Graph.hpp"

using Graph = dmf::Graph<uint32_t, uint32_t>;
using Node = dmf::Node<uint32_t>;
using Street = dmf::Street<uint32_t, uint32_t>;
using SparseMatrix = dmf::SparseMatrix<uint32_t, bool>;

using Bench = sb::Bench<long long int>;

int main() {
  Graph g1;
  const int n_rep{1000};
  Bench b1(n_rep);

  std::cout << "Benchmarking addNode\n";
  Node n1(std::rand());
  b1.benchmark([&g1](const Node& node) -> void { g1.addNode(node); }, n1);
  b1.print();
  std::cout << "Benchmarking addNodes overhead for a single node\n";
  n1 = Node(std::rand());
  b1.benchmark([&g1](const Node& node) -> void { g1.addNodes(node); }, n1);
  b1.print();

  std::cout << "Benchmarking addStreet\n";
  Street s1(std::rand());
  b1.benchmark([&g1](const Street& street) -> void { g1.addStreet(street); }, s1);
  b1.print();
  std::cout << "Benchmarking addStreets overhead for a single street\n";
  s1 = Street(std::rand());
  b1.benchmark([&g1](const Street& street) -> void { g1.addStreets(street); }, s1);
  b1.print();

  const int n_nodes{10000};
  SparseMatrix sm(n_nodes, n_nodes);
  std::mt19937 gen;
  std::uniform_real_distribution<> dis(0., 1.);

  for (int i{}; i < n_nodes * n_nodes; ++i) {
    if (dis(gen) < (4. / n_nodes)) {
      sm.insert(i, true);
    }
  }
  Bench b2;
  std::cout << "Benchmarking construction with adjacency matrix\n";
  b2.benchmark([&sm]() -> void { Graph g(sm); });
  b2.print<sb::milliseconds>();

  Bench b3(10);
  Graph g2(sm);
  std::cout << "Benchmarking building the adjacency matrix\n";
  b3.benchmark([&g2]() -> void { g2.buildAdj(); });
  b3.print<sb::microseconds>();
}
