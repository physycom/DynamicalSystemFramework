
#include <cstdint>
#include <iostream>
#include <random>
#include "Bench.hpp"

#include "RoadNetwork.hpp"

using Agent = dsf::Agent;
using Street = dsf::Street;
using SparseMatrix = dsf::SparseMatrix<bool>;

using Bench = sb::Bench<long long int>;

int main() {
  Street street(0, std::make_pair(0, 1), 5000.);
  Agent agent(0, 0, 0);
  Bench b(1000);

  // std::cout << "Benchmarking addAgent\n";
  // b.benchmark([&street](Agent ag) -> void { street.addAgent(ag.id()); }, agent);
  // b.print();

  // std::cout << "Benchmarking dequeue\n";
  // b.benchmark([&street]() -> void { street.dequeue(); });
  // b.print();
}
