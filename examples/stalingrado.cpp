/**
 * @brief Main function to simulate traffic dynamics over Via Stalingrado, in Bologna.
 */

#include "../src/dsf/dsf.hpp"
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

using RoadNetwork = dsf::RoadNetwork;
using Itinerary = dsf::Itinerary;
using Dynamics = dsf::FirstOrderDynamics;
using Street = dsf::Street;
using SpireStreet = dsf::SpireStreet;
using TrafficLight = dsf::TrafficLight;

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
  RoadNetwork graph;

  // Street(StreetId, Capacity, Length, vMax, (from, to))
  dsf::Road::setMeanVehicleLength(8.);
  Street s01{1, std::make_pair(0, 1), 2281., 13.9, 2};
  Street s12{7, std::make_pair(1, 2), 118., 13.9, 2};
  Street s23{13, std::make_pair(2, 3), 222., 13.9, 2};
  Street s34{19, std::make_pair(3, 4), 651., 13.9, 2};
  // Viale Aldo Moro
  graph.addNode<TrafficLight>(1, 132);
  auto& tl1 = graph.node<TrafficLight>(1);
  tl1.setCycle(s01.id(), dsf::Direction::ANY, {62, 0});
  // Via Donato Creti
  graph.addNode<TrafficLight>(2, 141);
  auto& tl2 = graph.node<TrafficLight>(2);
  tl2.setCycle(s12.id(), dsf::Direction::ANY, {72, 0});
  // Via del Lavoro
  graph.addNode<TrafficLight>(3, 138);
  auto& tl3 = graph.node<TrafficLight>(3);
  tl3.setCycle(s23.id(), dsf::Direction::ANY, {88, 0});
  // Viali
  graph.addNode<TrafficLight>(4, 131);
  auto& tl4 = graph.node<TrafficLight>(4);
  tl4.setCycle(s34.id(), dsf::Direction::ANY, {81, 0});

  graph.addStreets(s01, s12, s23, s34);
  graph.buildAdj();
  graph.adjustNodeCapacities();
  graph.makeSpireStreet(19);
  auto& spire = graph.edge<SpireStreet>(19);

  dsf::Logger::info(std::format("Intersections: {}", graph.nNodes()));
  dsf::Logger::info(std::format("Streets: {}", graph.nEdges()));

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
