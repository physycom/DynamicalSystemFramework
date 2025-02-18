/**
 * @brief Main function to simulate traffic dynamics over Via Stalingrado, in Bologna.
 */

#include "../src/dsm/dsm.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include <thread>
#ifdef __APPLE__
#define thread_t std::thread
#else
#define thread_t std::jthread
#endif
#include <atomic>

std::atomic<unsigned int> progress{0};

using Unit = unsigned int;
using Delay = uint8_t;

using Graph = dsm::Graph;
using Itinerary = dsm::Itinerary;
using Dynamics = dsm::FirstOrderDynamics;
using Street = dsm::Street;
using SpireStreet = dsm::SpireStreet;
using TrafficLight = dsm::TrafficLight;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main() {
  // Import input data
  std::ifstream ifs{"./data/stalingrado_input.txt"};
  Unit timeUnit{0};
  ifs >> timeUnit;
  std::vector<Unit> vehiclesToInsert{};
  while (!ifs.eof()) {
    Unit vehicleId{0};
    ifs >> vehicleId;
    vehiclesToInsert.push_back(vehicleId);
  }
  const auto MAX_TIME{static_cast<Unit>(timeUnit * vehiclesToInsert.size())};

  // Create the graph
  Graph graph;

  // Street(StreetId, Capacity, Length, vMax, (from, to))
  dsm::Road::setMeanVehicleLength(8.);
  Street s01{1, std::make_pair(0, 1), 2281., 13.9, 2};
  Street s12{7, std::make_pair(1, 2), 118., 13.9, 2};
  Street s23{13, std::make_pair(2, 3), 222., 13.9, 2};
  Street s34{19, std::make_pair(3, 4), 651., 13.9, 2};
  // Viale Aldo Moro
  auto& tl1 = graph.addNode<TrafficLight>(1, 132);
  tl1.setCycle(s01.id(), dsm::Direction::ANY, {62, 0});
  // Via Donato Creti
  auto& tl2 = graph.addNode<TrafficLight>(2, 141);
  tl2.setCycle(s12.id(), dsm::Direction::ANY, {72, 0});
  // Via del Lavoro
  auto& tl3 = graph.addNode<TrafficLight>(3, 138);
  tl3.setCycle(s23.id(), dsm::Direction::ANY, {88, 0});
  // Viali
  auto& tl4 = graph.addNode<TrafficLight>(4, 131);
  tl4.setCycle(s34.id(), dsm::Direction::ANY, {81, 0});

  graph.addStreets(s01, s12, s23, s34);
  graph.buildAdj();
  graph.adjustNodeCapacities();
  graph.makeSpireStreet(19);
  auto& spire = graph.edge<SpireStreet>(19);

  dsm::Logger::info(std::format("Intersections: {}", graph.nNodes()));
  dsm::Logger::info(std::format("Streets: {}", graph.nEdges()));

  // Create the dynamics
  Dynamics dynamics{graph, false, 69, 0.6};
  dynamics.setSpeedFluctuationSTD(0.2);
  dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(4, 4)));
  dynamics.updatePaths();

  // lauch progress bar
  thread_t t([MAX_TIME]() {
    while (progress < MAX_TIME) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });
  // Evolution
  auto it = vehiclesToInsert.begin();
  std::ofstream ofs{"./stalingrado_output.csv"};
  // print two columns, time and vehicles
  ofs << "time;vehicle_flux" << '\n';
  while (progress < MAX_TIME) {
    if (progress % 60 == 0) {
      if (progress != 0) {
        ++it;
      }
      if (progress % 300 == 0) {
        ofs << progress << ';' << spire.outputCounts(true) << std::endl;
      }
      dynamics.addAgents(*it, 4, 0);
    }
    dynamics.evolve(false);
    ++progress;
  }

#ifdef __APPLE__
  t.join();
#endif

  return 0;
}
