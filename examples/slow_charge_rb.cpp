#include "../src/dsf/dsf.hpp"
#include <array>
#include <chrono>
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

using Unit = unsigned int;
using Delay = uint8_t;

using RoadNetwork = dsf::RoadNetwork;
using Dynamics = dsf::FirstOrderDynamics;
using Street = dsf::Street;
using SpireStreet = dsf::SpireStreet;
using Roundabout = dsf::Roundabout;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char** argv) {
  auto const start = std::chrono::high_resolution_clock::now();
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <SEED> <ERROR_PROBABILITY> <OUT_FOLDER_BASE> <INIT_NAGENTS>\n";
    return 1;
  }

  const int SEED = std::stoi(argv[1]);  // seed for random number generator
  const double ERROR_PROBABILITY{std::stod(argv[2])};
  const std::string BASE_OUT_FOLDER{argv[3]};
  auto nAgents{std::stoul(argv[4])};

  std::cout << "-------------------------------------------------\n";
  std::cout << "Input parameters:\n";
  std::cout << "Seed: " << SEED << '\n';
  std::cout << "Error probability: " << ERROR_PROBABILITY << '\n';
  std::cout << "Base output folder: " << BASE_OUT_FOLDER << '\n';
  std::cout << "Initial number of agents: " << nAgents << '\n';
  std::cout << "-------------------------------------------------\n";

  const std::string IN_MATRIX{"./data/matrix.dat"};       // input matrix file
  const std::string IN_COORDS{"./data/coordinates.dsf"};  // input coords file
  const std::string OUT_FOLDER{std::format("{}output_scrb_{}_{}/",
                                           BASE_OUT_FOLDER,
                                           ERROR_PROBABILITY,
                                           std::to_string(SEED))};  // output folder
  constexpr auto MAX_TIME{static_cast<unsigned int>(5e5)};  // maximum time of simulation

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

  std::cout << "Number of nodes: " << graph.nNodes() << '\n';
  std::cout << "Number of streets: " << graph.nEdges() << '\n';

  std::cout << "Rounding the simulation...\n";
  for (Unit i{0}; i < graph.nNodes(); ++i) {
    graph.makeRoundabout(i);
  }
  std::cout << "Making every street a spire...\n";
  for (const auto& [streetId, pStreet] : graph.edges()) {
    graph.makeSpireStreet(streetId);
  }
  graph.adjustNodeCapacities();
  std::cout << "Done." << std::endl;

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph, true, SEED, 0.6};

  {
    std::vector<Unit> destinationNodes;
    for (auto const& [nodeId, pNode] : dynamics.graph().nodes()) {
      if (pNode->outgoingEdges().size() < 4) {
        destinationNodes.push_back(nodeId);
      }
    }
    dynamics.setDestinationNodes(destinationNodes);
    std::cout << "Number of exits: " << destinationNodes.size() << '\n';
  }
  dynamics.updatePaths();

  dynamics.setErrorProbability(0.05);
  dynamics.setPassageProbability(0.7707);
  // dynamics.setForcePriorities(true);
  dynamics.setSpeedFluctuationSTD(0.1);

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
  streetSpeed << "time;";
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
  for (const auto& [streetId, pStreet] : dynamics.graph().edges()) {
    outSpires << ';' << streetId;
    inSpires << ';' << streetId;
  }
  outSpires << '\n';
  inSpires << '\n';
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
  while (dynamics.time_step() < MAX_TIME) {
    if (dynamics.time_step() < MAX_TIME && dynamics.time_step() % 60 == 0) {
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

    if (dynamics.time_step() % 2400 == 0) {
      auto const totalDynamicsAgents{dynamics.nAgents()};
      deltaAgents = totalDynamicsAgents - previousAgents;
      if (deltaAgents < 0) {
        ++nAgents;
        std::cout << "- Now I'm adding " << nAgents << " agents.\n";
        std::cout << "Delta agents: " << deltaAgents << '\n';
        std::cout << "At time: " << dynamics.time_step() << '\n';
      }
      previousAgents = totalDynamicsAgents;
    }

    if (dynamics.time_step() % 300 == 0) {
#ifdef PRINT_OUT_SPIRES
      outSpires << dynamics.time_step();
      inSpires << dynamics.time_step();
      for (const auto& [streetId, pStreet] : dynamics.graph().edges()) {
        auto& spire = dynamic_cast<SpireStreet&>(*pStreet);
        outSpires << ';' << spire.outputCounts(false);
        inSpires << ';' << spire.inputCounts(false);
      }
      outSpires << std::endl;
      inSpires << std::endl;
#endif
      printLoadingBar(dynamics.time_step(), MAX_TIME);
      dynamics.saveMacroscopicObservables(std::format("{}data.csv", OUT_FOLDER));
    }
    if (dynamics.time_step() % 10 == 0) {
#ifdef PRINT_DENSITIES
      dynamics.saveStreetDensities(OUT_FOLDER + "densities.csv", true);
#endif
#ifdef PRINT_FLOWS
      streetFlow << dynamics.time_step();
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
      streetSpeed << dynamics.time_step();
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
  std::cout << "Total elapsed time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start)
                   .count()
            << " milliseconds\n";
  return 0;
}
