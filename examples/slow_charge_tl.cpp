#include "../src/dsf/dsf.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <format>
#include <filesystem>
namespace fs = std::filesystem;

#include <thread>
#ifdef __APPLE__
#define thread_t std::thread
#else
#define thread_t std::jthread
#endif
#include <atomic>

std::atomic<unsigned int> progress{0};
std::atomic<bool> bExitFlag{false};

// uncomment these lines to print densities, flows and speeds
#define PRINT_DENSITIES
// #define PRINT_FLOWS
#define PRINT_OUT_SPIRES
// #define PRINT_SPEEDS
// #define PRINT_TP

using Unit = unsigned int;
using Delay = uint8_t;

using RoadNetwork = dsf::RoadNetwork;
using Dynamics = dsf::FirstOrderDynamics;
using Street = dsf::Street;
using SpireStreet = dsf::SpireStreet;
using TrafficLight = dsf::TrafficLight;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr
        << "Usage: " << argv[0]
        << " <SEED> <ERROR_PROBABILITY> <OUT_FOLDER_BASE> <OPTIMIZE> <INIT_NAGENTS>\n";
    return 1;
  }

  const int SEED = std::stoi(argv[1]);  // seed for random number generator
  const double ERROR_PROBABILITY{std::stod(argv[2])};
  std::string BASE_OUT_FOLDER{argv[3]};
  const bool OPTIMIZE{std::string(argv[4]) != std::string("0")};
  BASE_OUT_FOLDER += OPTIMIZE ? "_op/" : "/";
  auto nAgents{std::stoul(argv[5])};

  const std::string IN_MATRIX{"./data/matrix.dat"};       // input matrix file
  const std::string IN_COORDS{"./data/coordinates.dsf"};  // input coords file
  std::string OUT_FOLDER{std::format("{}output_sctl_{}_{}/",
                                     BASE_OUT_FOLDER,
                                     ERROR_PROBABILITY,
                                     std::to_string(SEED))};  // output folder
  constexpr auto MAX_TIME{static_cast<unsigned int>(5e5)};  // maximum time of simulation

  std::cout << "-------------------------------------------------\n";
  std::cout << "Input parameters:\n";
  std::cout << "Seed: " << SEED << '\n';
  std::cout << "Error probability: " << ERROR_PROBABILITY << '\n';
  std::cout << "Base output folder: " << BASE_OUT_FOLDER << '\n';
  std::cout << "Initial number of agents: " << nAgents << '\n';
  if (OPTIMIZE) {
    std::cout << "Traffic light optimization ENABLED.\n";
  }
  std::cout << "-------------------------------------------------\n";

  // Clear output folder or create it if it doesn't exist
  if (!fs::exists(BASE_OUT_FOLDER)) {
    fs::create_directory(BASE_OUT_FOLDER);
  }
  if (fs::exists(OUT_FOLDER)) {
    fs::remove_all(OUT_FOLDER);
  }
  fs::create_directory(OUT_FOLDER);
  // Starting
  std::cout << "Using dsf version: " << dsf::version() << '\n';
  RoadNetwork graph{};
  std::cout << "Importing matrix.dat...\n";
  graph.importMatrix(IN_MATRIX, false);
  graph.importCoordinates(IN_COORDS);
  std::cout << "Setting street parameters..." << '\n';

  // graph.addStreet(Street(100002, std::make_pair(0, 108)));
  // graph.addStreet(Street(100003, std::make_pair(108, 0)));
  // graph.addStreet(Street(100004, std::make_pair(0, 11)));
  // graph.addStreet(Street(100005, std::make_pair(11, 0)));
  // graph.addStreet(Street(100006, std::make_pair(1, 109)));
  // graph.addStreet(Street(100007, std::make_pair(109, 1)));
  // graph.addStreet(Street(100008, std::make_pair(2, 110)));
  // graph.addStreet(Street(100009, std::make_pair(110, 2)));
  // graph.addStreet(Street(100010, std::make_pair(3, 111)));
  // graph.addStreet(Street(100011, std::make_pair(111, 3)));
  // graph.addStreet(Street(100012, std::make_pair(4, 112)));
  // graph.addStreet(Street(100013, std::make_pair(112, 4)));
  // graph.addStreet(Street(100014, std::make_pair(5, 113)));
  // graph.addStreet(Street(100015, std::make_pair(113, 5)));
  // graph.addStreet(Street(100016, std::make_pair(6, 114)));
  // graph.addStreet(Street(100017, std::make_pair(114, 6)));
  // graph.addStreet(Street(100018, std::make_pair(7, 115)));
  // graph.addStreet(Street(100019, std::make_pair(115, 7)));
  // graph.addStreet(Street(100020, std::make_pair(8, 116)));
  // graph.addStreet(Street(100021, std::make_pair(116, 8)));
  // graph.addStreet(Street(100022, std::make_pair(9, 117)));
  // graph.addStreet(Street(100023, std::make_pair(117, 9)));
  // graph.addStreet(Street(100024, std::make_pair(10, 118)));
  // graph.addStreet(Street(100025, std::make_pair(118, 10)));
  // graph.addStreet(Street(100026, std::make_pair(11, 119)));
  // graph.addStreet(Street(100027, std::make_pair(119, 11)));
  // graph.addStreet(Street(100028, std::make_pair(12, 23)));
  // graph.addStreet(Street(100029, std::make_pair(23, 12)));
  // graph.addStreet(Street(100030, std::make_pair(24, 35)));
  // graph.addStreet(Street(100069, std::make_pair(35, 24)));
  // graph.addStreet(Street(100031, std::make_pair(36, 47)));
  // graph.addStreet(Street(100032, std::make_pair(47, 36)));
  // graph.addStreet(Street(100033, std::make_pair(48, 59)));
  // graph.addStreet(Street(100034, std::make_pair(59, 48)));
  // graph.addStreet(Street(100035, std::make_pair(60, 71)));
  // graph.addStreet(Street(100036, std::make_pair(71, 60)));
  // graph.addStreet(Street(100037, std::make_pair(72, 83)));
  // graph.addStreet(Street(100038, std::make_pair(83, 72)));
  // graph.addStreet(Street(100039, std::make_pair(84, 95)));
  // graph.addStreet(Street(100040, std::make_pair(95, 84)));
  // graph.addStreet(Street(100041, std::make_pair(96, 107)));
  // graph.addStreet(Street(100042, std::make_pair(107, 96)));
  // graph.addStreet(Street(100043, std::make_pair(108, 119)));
  // graph.addStreet(Street(100044, std::make_pair(119, 108)));

  // graph.buildAdj();

  std::cout << "Number of nodes: " << graph.nNodes() << '\n';
  std::cout << "Number of streets: " << graph.nEdges() << '\n';

  std::cout << "Traffic Lightning the simulation...\n";
  for (Unit i{0}; i < graph.nNodes(); ++i) {
    graph.makeTrafficLight(i, 120);
  }
  std::cout << "Making every street a spire...\n";
  for (const auto& pStreet : graph.edges()) {
    graph.makeSpireStreet(pStreet->id());
  }
  // create gaussian random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  gen.seed(64313);
  std::normal_distribution d(60., 10.);
  std::array<uint8_t, 2> sda{0, 0};
  auto random = [&d, &gen]() { return std::round(d(gen)); };
  std::cout << "Setting traffic light parameters..." << '\n';
  graph.adjustNodeCapacities();
  for (const auto& pNode : graph.nodes()) {
    auto& tl = dynamic_cast<TrafficLight&>(*pNode);
    double value = -1.;
    while (value < 0.) {
      value = random();
    }
    const auto& col = graph.inputNeighbors(pNode->id());
    std::set<Unit> streets;
    const auto& refLat = pNode->coords().value().first;
    for (const auto& pInputNode : col) {
      const auto& lat = pInputNode->coords().value().first;
      // std::cout << "Lat: " << lat << " RefLat: " << refLat << '\n';
      if (lat == refLat) {
        streets.emplace(graph.edge(pInputNode->id(), pNode->id())->id());
      }
    }
    for (auto const& streetId : streets) {
      tl.setCycle(streetId, dsf::Direction::ANY, {static_cast<dsf::Delay>(value), 0});
    }
    for (const auto& pInputNode : col) {
      auto const streetId = graph.edge(pInputNode->id(), pNode->id())->id();
      if (!streets.contains(streetId)) {
        tl.setComplementaryCycle(streetId, *streets.begin());
      }
    }
    ++sda[streets.size() - 1];
    // std::cout << "Node id: " << nodeId << " has " << streets.size()
    //           << "streets.\n";
  }
  std::cout << "Nodes with one street: " << static_cast<int>(sda[0]) << '\n';
  std::cout << "Nodes with two streets: " << static_cast<int>(sda[1]) << '\n';
  std::cout << "Done." << std::endl;

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph, true, SEED, 0.6};
  Unit n{0};
  {
    std::vector<Unit> destinationNodes;
    for (auto const& pNode : graph.nodes()) {
      if (graph.outputNeighbors(pNode->id()).size() < 4) {
        destinationNodes.push_back(pNode->id());
        ++n;
      }
    }
    dynamics.setDestinationNodes(destinationNodes);
  }
  std::cout << "Number of exits: " << n << '\n';

  dynamics.setErrorProbability(ERROR_PROBABILITY);
  // dynamics.setMaxFlowPercentage(0.69);
  // dynamics.setForcePriorities(false);
  dynamics.setSpeedFluctuationSTD(0.1);
  if (OPTIMIZE)
    dynamics.setDataUpdatePeriod(30);  // Store data every 30 time steps

  const auto TM = dynamics.turnMapping();

  std::cout << "Done." << std::endl;
  std::cout << "Running simulation...\n";
#ifdef PRINT_FLOWS
  std::ofstream streetFlow(OUT_FOLDER + "flows.csv");
  streetFlow << "time";
  for (const auto& [id, street] : dynamics.graph().edges()) {
    streetFlow << ';' << id;
  }
  streetFlow << '\n';
#endif
#ifdef PRINT_SPEEDS
  std::ofstream streetSpeed(OUT_FOLDER + "speeds.csv");
  streetSpeed << "time";
  for (const auto& [id, street] : dynamics.graph().edges()) {
    streetSpeed << ';' << id;
  }
  streetSpeed << '\n';
#endif
#ifdef PRINT_OUT_SPIRES
  std::ofstream outSpires(OUT_FOLDER + "out_spires.csv");
  std::ofstream inSpires(OUT_FOLDER + "in_spires.csv");
  outSpires << "time";
  inSpires << "time";
  for (const auto& pStreet : dynamics.graph().edges()) {
    outSpires << ';' << pStreet->id();
    inSpires << ';' << pStreet->id();
  }
  outSpires << '\n';
  inSpires << '\n';
#endif
#ifdef PRINT_TP
  std::ofstream outTP(OUT_FOLDER + "turn_probabilities.csv");
  outTP << "time";
  for (const auto& [id, street] : dynamics.graph().edges()) {
    outTP << ';' << id;
  }
  outTP << '\n';
#endif

  int deltaAgents{std::numeric_limits<int>::max()};
  int previousAgents{0};

  // std::vector<int> deltas;

  // lauch progress bar
  thread_t t([]() {
    while (progress < MAX_TIME && !bExitFlag) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
  });
  // dynamics.addAgentsUniformly(20000);
  while (dynamics.time() < MAX_TIME) {
    if (dynamics.time() < MAX_TIME && nAgents > 0 && dynamics.time() % 60 == 0) {
      try {
        dynamics.addAgentsUniformly(nAgents);
      } catch (const std::overflow_error& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Overflow reached. Exiting the simulation..." << std::endl;
        bExitFlag = true;
        break;
      }
    }
    dynamics.evolve(false);
    if (OPTIMIZE && (dynamics.time() % 420 == 0)) {
      dynamics.optimizeTrafficLights(
          dsf::TrafficLightOptimization::DOUBLE_TAIL, std::string(), 0.3);
    }
    if (dynamics.time() % 2400 == 0 && nAgents > 0) {
      // auto meanDelta = std::accumulate(deltas.begin(), deltas.end(), 0) /
      // deltas.size();
      auto const totalDynamicsAgents{dynamics.nAgents()};
      deltaAgents = totalDynamicsAgents - previousAgents;
      if (deltaAgents < 0) {
        ++nAgents;
        std::cout << "- Now I'm adding " << nAgents << " agents.\n";
        std::cout << "Delta agents: " << deltaAgents << '\n';
        std::cout << "At time: " << dynamics.time() << '\n';
      }
      previousAgents = totalDynamicsAgents;
      // deltas.clear();
    }

    // if (meanDensity.mean > 0.7) {
    //   nAgents = 0;
    // }

    if (dynamics.time() % 300 == 0) {
      // printLoadingBar(dynamics.time(), MAX_TIME);
      // deltaAgents = std::labs(dynamics.agents().size() - previousAgents);
#ifdef PRINT_OUT_SPIRES
      outSpires << dynamics.time();
      inSpires << dynamics.time();
      for (const auto& pStreet : dynamics.graph().edges()) {
        auto& spire = dynamic_cast<SpireStreet&>(*pStreet);
        outSpires << ';' << spire.outputCounts(false);
        inSpires << ';' << spire.inputCounts(false);
      }
      outSpires << std::endl;
      inSpires << std::endl;
#endif
      dynamics.saveMacroscopicObservables(std::format("{}data.csv", OUT_FOLDER));
      // deltas.push_back(deltaAgents);
      // previousAgents = dynamics.agents().size();
#ifdef PRINT_TP
      const auto& tc{dynamics.turnCounts()};
      outTP << dynamics.time();
      for (const auto& [id, street] : dynamics.graph().edges()) {
        const auto& probs{tc.at(id)};
        outTP << ";[";
        const auto& nextStreets = TM.at(id);
        bool first = true;
        for (auto i = 0; i < 4; ++i) {
          if (nextStreets[i] < 0) {
            continue;
          }
          if (!first) {
            outTP << ',';
          }
          outTP << '(';
          outTP << nextStreets[i] << ',';
          outTP << probs[i];
          outTP << ')';
          first = false;
        }
        // for (const auto& prob: probs) {
        //   outTP << prob;
        //   if (&prob != &probs.back()) {
        //     outTP << ',';
        //   }
        // }
        outTP << ']';
      }
      outTP << std::endl;
#endif
    }
    if (dynamics.time() % 10 == 0) {
#ifdef PRINT_DENSITIES
      dynamics.saveStreetDensities(OUT_FOLDER + "densities.csv", true);
#endif
#ifdef PRINT_FLOWS
      streetFlow << ';' << dynamics.time();
      for (const auto& [id, street] : dynamics.graph().edges()) {
        const auto& meanSpeed = dynamics.streetMeanSpeed(id);
        if (meanSpeed.has_value()) {
          streetFlow << ';' << meanSpeed.value() * street->density();
        } else {
          streetFlow << ';';
        }
      }
      streetFlow << std::endl;
#endif
#ifdef PRINT_SPEEDS
      streetSpeed << dynamics.time();
      for (const auto& [id, street] : dynamics.graph().edges()) {
        const auto& meanSpeed = dynamics.streetMeanSpeed(id);
        if (meanSpeed.has_value()) {
          streetSpeed << ';' << meanSpeed.value();
        } else {
          streetSpeed << ';';
        }
      }
      streetSpeed << std::endl;
#endif
    }
    ++progress;
  }
#ifdef PRINT_FLOWS
  streetFlow.close();
#endif
#ifdef PRINT_SPEEDS
  streetSpeed.close();
#endif
#ifdef PRINT_OUT_SPIRES
  outSpires.close();
  inSpires.close();
#endif
  // std::cout << std::endl;
  // std::map<uint8_t, std::string> turnNames{
  //     {0, "left"}, {1, "straight"}, {2, "right"}, {3, "u-turn"}};
  // const auto prob = dynamics.turnProbabilities();
  // uint8_t i{0};
  // for (auto value: prob) {
  //   std::cout << "Probability of turning " << std::quoted(turnNames[i]) << ": " << value * 100 << "%\n";
  //   ++i;
  // }
  std::cout << '\n';
  std::cout << "Done." << std::endl;

#ifdef __APPLE__
  t.join();
#endif

  return 0;
}
