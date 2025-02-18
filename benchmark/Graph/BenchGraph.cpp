
#include <cstdint>
#include <iostream>
#include <random>
#include <utility>
#include "Bench.hpp"

#include "RoadNetwork.hpp"

using RoadNetwork = dsm::RoadNetwork;
using Intersection = dsm::Intersection;
using Street = dsm::Street;
using SparseMatrix = dsm::SparseMatrix<bool>;

using Bench = sb::Bench<long long int>;

int main() {
  RoadNetwork g1;
  const int n_rep{1000};
  Bench b1(n_rep);

  std::cout << "Benchmarking addNode\n";
  b1.benchmark([&g1]() -> void { g1.addNode<Intersection>(std::rand()); });
  b1.print();
  std::cout << "Benchmarking addNodes overhead for a single node\n";
  // n1 = Intersection(std::rand());
  // b1.benchmark([&g1](const Intersection& node) -> void { g1.addNodes(node); }, n1);
  // b1.print();

  // std::cout << "Benchmarking addStreet\n";
  // Street s1(std::rand(), std::make_pair(0, 1));
  // b1.benchmark([&g1](const Street& street) -> void { g1.addStreet(street); }, s1);
  // b1.print();
  // std::cout << "Benchmarking addStreets overhead for a single street\n";
  // s1 = Street(std::rand(), std::make_pair(0, 1));
  // b1.benchmark([&g1](const Street& street) -> void { g1.addStreets(street); }, s1);
  // b1.print();

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
  b2.benchmark([&sm]() -> void { RoadNetwork g(sm); });
  b2.print<sb::milliseconds>();

  // Bench b3(1);
  // RoadNetwork g2(sm);
  // std::cout << "Benchmarking building the adjacency matrix\n";
  // b3.benchmark([&g2]() -> void { g2.buildAdj(); });
  // b3.print<sb::microseconds>();

  // Bench b4(3);
  // RoadNetwork g3;
  // g3.importMatrix("./RoadNetwork/data/matrix.dat");
  // std::cout << "Benchmarking the algorithm for the shortest path\n";
  // b4.benchmark([&g3]() -> void { g3.shortestPath(0, 1); });
  // b4.print<sb::microseconds>();
}
