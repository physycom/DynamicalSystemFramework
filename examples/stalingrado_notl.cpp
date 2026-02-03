/**
 * @brief Main function to simulate traffic dynamics over Via Stalingrado, in Bologna.
 * In this case, traffic lights are removed.
 */

#include <dsf/dsf.hpp>
#include <array>
#include <cstdint>
#include <format>
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

constexpr std::size_t SAVE_INTERVAL = 300;

using namespace dsf::mobility;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char* argv[]) {
  const auto SCALING_FACTOR = std::stod(argv[1]);
  const auto TIME_SCALING = std::stod(argv[2]);
  // Import input data
  std::ifstream ifs{"./data/stalingrado_input.txt"};
  std::size_t timeUnit{0};
  ifs >> timeUnit;
  std::vector<std::size_t> vehiclesToInsert{};
  while (!ifs.eof()) {
    std::size_t vehicleId{0};
    ifs >> vehicleId;
    vehiclesToInsert.push_back(vehicleId);
  }
  const auto MAX_TIME{static_cast<std::size_t>(timeUnit * vehiclesToInsert.size())};

  // Create the graph
  RoadNetwork graph;

  // Street(StreetId, Capacity, Length, vMax, (from, to))
  dsf::mobility::Road::setMeanVehicleLength(5.);
  Street s01{1, std::make_pair(0, 1), 2281., 13.9, 2};
  Street s12{7, std::make_pair(1, 2), 118., 13.9, 2};
  Street s23{13, std::make_pair(2, 3), 222., 13.9, 2};
  Street s34{19, std::make_pair(3, 4), 651., 13.9, 2};
  // Viale Aldo Moro
  // graph.addNode<TrafficLight>(1, 132);
  // auto& tl1 = graph.node<TrafficLight>(1);
  // tl1.setCycle(s01.id(), dsf::Direction::ANY, {62, 0});
  // Via Donato Creti
  // graph.addNode<TrafficLight>(2, 141);
  // auto& tl2 = graph.node<TrafficLight>(2);
  // tl2.setCycle(s12.id(), dsf::Direction::ANY, {72, 0});
  // Via del Lavoro
  // graph.addNode<TrafficLight>(3, 138);
  // auto& tl3 = graph.node<TrafficLight>(3);
  // tl3.setCycle(s23.id(), dsf::Direction::ANY, {88, 0});
  // Viali
  // graph.addNode<TrafficLight>(4, 131);
  // auto& tl4 = graph.node<TrafficLight>(4);
  // tl4.setCycle(s34.id(), dsf::Direction::ANY, {81, 0});

  graph.addStreets(s01, s12, s23, s34);
  graph.adjustNodeCapacities();

  graph.edge(1)->setTransportCapacity((62. / 132.) * TIME_SCALING);
  graph.edge(7)->setTransportCapacity((72. / 141.) * TIME_SCALING);
  graph.edge(13)->setTransportCapacity((88. / 138.) * TIME_SCALING);
  graph.edge(19)->setTransportCapacity((81. / 131.) * TIME_SCALING);

  graph.addCoil(19);
  auto const& coil = graph.edge(19);

  // Create the dynamics
  FirstOrderDynamics dynamics{graph, false, 69, 0.8};
  // dynamics.setSpeedFluctuationSTD(0.2);
  dynamics.addItinerary(std::make_shared<Itinerary>(4, 4));
  dynamics.updatePaths();

  auto pItinerary = dynamics.itineraries().at(4);

  // lauch progress bar
  thread_t t([MAX_TIME]() {
    while (progress < MAX_TIME) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });
  // Evolution
  // Evolution
  auto it = vehiclesToInsert.begin();
  std::ofstream ofs{std::format(
      "./stalingrado_output_notl_{:.1f}_{:.2f}.csv", SCALING_FACTOR, TIME_SCALING)};
  std::ofstream ofs_queues{std::format(
      "./stalingrado_notl_queues_{:.1f}_{:.2f}.csv", SCALING_FACTOR, TIME_SCALING)};
  // print two columns, time and vehicles
  ofs << "time;vehicle_flux\n";
  ofs_queues << "time;1;7;13;19\n";

  auto const& edges{dynamics.graph().edges()};

  while (progress < MAX_TIME) {
    if (progress % 60 == 0) {
      if (progress != 0) {
        ++it;
      }
      if (progress % SAVE_INTERVAL == 0) {
        ofs << progress << ';' << coil->counts() << std::endl;
        coil->resetCounter();
      }
      dynamics.addAgents(
          static_cast<std::size_t>(std::round(*it * SCALING_FACTOR)), pItinerary, 0);
    }
    if (progress % 30 == 0) {
      ofs_queues << progress << ';' << edges.at(1)->nExitingAgents() << ';'
                 << edges.at(7)->nExitingAgents() << ';' << edges.at(13)->nExitingAgents()
                 << ';' << edges.at(19)->nExitingAgents() << std::endl;
    }
    dynamics.evolve(false);
    ++progress;
  }

#ifdef __APPLE__
  t.join();
#endif

  return 0;
}
