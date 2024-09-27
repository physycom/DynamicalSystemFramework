#include "../src/dsm/dsm.hpp"
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
#include <filesystem>
namespace fs = std::filesystem;

#include <thread>
#include <atomic>

std::atomic<unsigned int> progress{0};
uint nAgents{315};  // 315 for error probability 0.3, 450 for error probability 0.05

// uncomment these lines to print densities, flows and speeds
#define PRINT_DENSITIES
// #define PRINT_FLOWS
#define PRINT_OUT_SPIRES
// #define PRINT_SPEEDS
// #define PRINT_TP

using Unit = unsigned int;
using Delay = uint8_t;

using Graph = dsm::Graph<Unit, Unit>;
using Itinerary = dsm::Itinerary<Unit>;
using Dynamics = dsm::FirstOrderDynamics<Unit, Unit, Delay>;
using Street = dsm::Street<Unit, Unit>;
using SpireStreet = dsm::SpireStreet<Unit, Unit>;
using TrafficLight = dsm::TrafficLight<Unit, Unit, Delay>;

void printLoadingBar(int const i, int const n) {
  std::cout << "Loading: " << std::setprecision(2) << std::fixed << (i * 100. / n) << "%"
            << '\r';
  std::cout.flush();
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " <SEED> <ERROR_PROBABILITY> <OUT_FOLDER_BASE> <OPTIMIZE>\n";
    return 1;
  }

  const int SEED = std::stoi(argv[1]);  // seed for random number generator
  const double ERROR_PROBABILITY{std::stod(argv[2])};
  const std::string BASE_OUT_FOLDER{argv[3]};
  const bool OPTIMIZE{std::string(argv[4]) != std::string("0")};

  std::cout << "-------------------------------------------------\n";
  std::cout << "Input parameters:\n";
  std::cout << "Seed: " << SEED << '\n';
  std::cout << "Error probability: " << ERROR_PROBABILITY << '\n';
  std::cout << "Base output folder: " << BASE_OUT_FOLDER << '\n';
  if (OPTIMIZE) {
    std::cout << "Traffic light optimization enabled.\n";
  }
  std::cout << "-------------------------------------------------\n";

  const std::string IN_MATRIX{"./data/matrix.dat"};       // input matrix file
  const std::string IN_COORDS{"./data/coordinates.dsm"};  // input coords file
  std::string OUT_FOLDER{BASE_OUT_FOLDER + "output_sctl_0.3_" +
                         std::to_string(SEED)};  // output folder
  if (OPTIMIZE) {
    OUT_FOLDER += "_op/";
  }
  const auto MAX_TIME{static_cast<unsigned int>(1e6)};  // maximum time of simulation

  // Clear output folder or create it if it doesn't exist
  if (!fs::exists(BASE_OUT_FOLDER)) {
    fs::create_directory(BASE_OUT_FOLDER);
  }
  if (fs::exists(OUT_FOLDER)) {
    fs::remove_all(OUT_FOLDER);
  }
  fs::create_directory(OUT_FOLDER);
  // Starting
  std::cout << "Using dsm version: " << dsm::version() << '\n';
  Graph graph{};
  std::cout << "Importing matrix.dat...\n";
  graph.importMatrix(IN_MATRIX, false);
  graph.importCoordinates(IN_COORDS);
  graph.buildAdj();
  const auto dv = graph.adjMatrix().getDegreeVector();

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

  std::cout << "Number of nodes: " << graph.nodeSet().size() << '\n';
  std::cout << "Number of streets: " << graph.streetSet().size() << '\n';

  std::cout << "Traffic Lightning the simulation...\n";
  for (Unit i{0}; i < graph.nodeSet().size(); ++i) {
    graph.makeTrafficLight<Delay>(i);
  }
  std::cout << "Making every street a spire...\n";
  for (const auto& [id, street] : graph.streetSet()) {
    graph.makeSpireStreet(id);
  }
  // check isSpire for each street
  for (const auto& [id, street] : graph.streetSet()) {
    if (!street->isSpire()) {
      std::cerr << "Street " << id << " is not a spire.\n";
    }
  }
  std::cout << "Setting street parameters..." << '\n';
  for (const auto& [streetId, street] : graph.streetSet()) {
    street->setLength(2e3);
    street->setCapacity(225);
    street->setTransportCapacity(1);
    street->setMaxSpeed(13.9);
  }
  const auto& adj = graph.adjMatrix();
  const auto& degreeVector = adj.getDegreeVector();
  // create gaussian random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  gen.seed(64313);
  std::normal_distribution d(60., 10.);
  std::array<uint8_t, 2> sda{0, 0};
  auto random = [&d, &gen]() { return std::round(d(gen)); };
  std::cout << "Setting traffic light parameters..." << '\n';
  for (const auto& [nodeId, node] : graph.nodeSet()) {
    auto& tl = dynamic_cast<TrafficLight&>(*node);
    tl.setCapacity(degreeVector(nodeId));
    double value = -1.;
    while (value < 0.) {
      value = random();
    }
    tl.setDelay(static_cast<Delay>(value));
    const auto& col = adj.getCol(nodeId, true);
    std::set<Unit> streets;
    const auto id = col.begin();
    const auto& refLat = graph.nodeSet()
                             .at(graph.streetSet().at(id->first)->nodePair().second)
                             ->coords()
                             .value()
                             .first;
    for (const auto& [c, value] : col) {
      const auto& lat = graph.nodeSet()
                            .at(graph.streetSet().at(c)->nodePair().first)
                            ->coords()
                            .value()
                            .first;
      // std::cout << "Lat: " << lat << " RefLat: " << refLat << '\n';
      if (lat == refLat) {
        streets.emplace(c);
      }
    }
    tl.setStreetPriorities(streets);
    ++sda[streets.size() - 1];
    // std::cout << "Node id: " << nodeId << " has " << streets.size()
    //           << "streets.\n";
  }
  std::cout << "Nodes with one street: " << static_cast<int>(sda[0]) << '\n';
  std::cout << "Nodes with two streets: " << static_cast<int>(sda[1]) << '\n';
  std::cout << "Done." << std::endl;

  std::cout << "Creating dynamics...\n";

  Dynamics dynamics{graph};
  Unit n{0};
  for (const auto& [nodeId, degree] : dv) {
    if (degree < 4) {
      dynamics.addItinerary(Itinerary{n, nodeId});
      ++n;
    }
  }
  std::cout << "Number of exits: " << n << '\n';

  dynamics.setSeed(SEED);
  dynamics.setErrorProbability(ERROR_PROBABILITY);
  // dynamics.setMaxFlowPercentage(0.69);
  // dynamics.setForcePriorities(false);
  dynamics.setSpeedFluctuationSTD(0.1);
  dynamics.setMinSpeedRateo(0.95);
  dynamics.updatePaths();

  const auto TM = dynamics.turnMapping();

  std::cout << "Done." << std::endl;
  std::cout << "Running simulation...\n";

  std::ofstream out(OUT_FOLDER + "data.csv");
  out << "time;n_agents;mean_speed;mean_speed_err;mean_density;mean_density_"
         "err;mean_flow;mean_flow_err;mean_traveltime;mean_traveltime_err;mean_flow_"
         "spires;mean_flow_spires_err\n";
#ifdef PRINT_DENSITIES
  std::ofstream streetDensity(OUT_FOLDER + "densities.csv");
  streetDensity << "time;";
  for (const auto& [id, street] : dynamics.graph().streetSet()) {
    streetDensity << id << ';';
  }
  streetDensity << '\n';
#endif
#ifdef PRINT_FLOWS
  std::ofstream streetFlow(OUT_FOLDER + "flows.csv");
  streetFlow << "time;";
  for (const auto& [id, street] : dynamics.graph().streetSet()) {
    streetFlow << id << ';';
  }
  streetFlow << '\n';
#endif
#ifdef PRINT_SPEEDS
  std::ofstream streetSpeed(OUT_FOLDER + "speeds.csv");
  streetSpeed << "time;";
  for (const auto& [id, street] : dynamics.graph().streetSet()) {
    streetSpeed << id << ';';
  }
  streetSpeed << '\n';
#endif
#ifdef PRINT_OUT_SPIRES
  std::ofstream outSpires(OUT_FOLDER + "out_spires.csv");
  std::ofstream inSpires(OUT_FOLDER + "in_spires.csv");
  outSpires << "time;";
  inSpires << "time;";
  for (const auto& [id, street] : dynamics.graph().streetSet()) {
    outSpires << id << ';';
    inSpires << id << ';';
  }
  outSpires << '\n';
  inSpires << '\n';
#endif
#ifdef PRINT_TP
  std::ofstream outTP(OUT_FOLDER + "turn_probabilities.csv");
  outTP << "time;";
  for (const auto& [id, street] : dynamics.graph().streetSet()) {
    outTP << id << ';';
  }
  outTP << '\n';
#endif

  int deltaAgents{std::numeric_limits<int>::max()};
  int previousAgents{0};

  // std::vector<int> deltas;

  // lauch progress bar
  std::thread t([]() {
    while (progress < MAX_TIME) {
      printLoadingBar(progress, MAX_TIME);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  // dynamics.addAgentsUniformly(20000);
  while (dynamics.time() < MAX_TIME) {
    if (dynamics.time() < MAX_TIME && nAgents > 0) {
      if (dynamics.time() % 60 == 0) {
        dynamics.addAgentsUniformly(nAgents);
      }
    }
    dynamics.evolve(false);
    if (OPTIMIZE && (dynamics.time() % 420 == 0)) {
      dynamics.optimizeTrafficLights(std::floor(420. / 60), 0.15);
    }
    if (dynamics.time() % 2400 == 0 && nAgents > 0) {
      // auto meanDelta = std::accumulate(deltas.begin(), deltas.end(), 0) /
      // deltas.size();
      deltaAgents = dynamics.agents().size() - previousAgents;
      if (deltaAgents < 0) {
        ++nAgents;
        std::cout << "- Now I'm adding " << nAgents << " agents.\n";
        std::cout << "Delta agents: " << deltaAgents << '\n';
        std::cout << "At time: " << dynamics.time() << '\n';
      }
      previousAgents = dynamics.agents().size();
      // deltas.clear();
    }

    // if (meanDensity.mean > 0.7) {
    //   nAgents = 0;
    // }

    if (dynamics.time() % 300 == 0) {
      // printLoadingBar(dynamics.time(), MAX_TIME);
      // deltaAgents = std::labs(dynamics.agents().size() - previousAgents);
#ifdef PRINT_OUT_SPIRES
      outSpires << dynamics.time() << ';';
      inSpires << dynamics.time() << ';';
      for (const auto& [id, street] : dynamics.graph().streetSet()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        outSpires << spire.outputCounts(false) << ';';
        inSpires << spire.inputCounts(false) << ';';
      }
      outSpires << std::endl;
      inSpires << std::endl;
#endif
      const auto& meanSpeed{dynamics.streetMeanSpeed()};
      const auto& meanDensity{dynamics.streetMeanDensity()};
      const auto& meanFlow{dynamics.streetMeanFlow()};
      const auto& meanTravelTime{dynamics.meanTravelTime()};
      const auto& meanSpireFlow{dynamics.meanSpireOutputFlow()};

      out << dynamics.time() << ';' << dynamics.agents().size() << ';' << meanSpeed.mean
          << ';' << meanSpeed.std << ';' << meanDensity.mean << ';' << meanDensity.std
          << ';' << meanFlow.mean << ';' << meanFlow.std << ';' << meanTravelTime.mean
          << ';' << meanTravelTime.std << ';' << meanSpireFlow.mean << ';'
          << meanSpireFlow.std << std::endl;
      // deltas.push_back(deltaAgents);
      // previousAgents = dynamics.agents().size();
#ifdef PRINT_TP
      const auto& tc{dynamics.turnCounts()};
      outTP << dynamics.time() << ';';
      for (const auto& [id, street] : dynamics.graph().streetSet()) {
        const auto& probs{tc.at(id)};
        outTP << '[';
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
        outTP << "];";
      }
      outTP << std::endl;
#endif
    }
    if (dynamics.time() % 10 == 0) {
#ifdef PRINT_DENSITIES
      streetDensity << dynamics.time() << ';';
      for (const auto& [id, street] : dynamics.graph().streetSet()) {
        streetDensity << street->density() << ';';
      }
      streetDensity << std::endl;
#endif
#ifdef PRINT_FLOWS
      streetFlow << dynamics.time() << ';';
      for (const auto& [id, street] : dynamics.graph().streetSet()) {
        const auto& meanSpeed = dynamics.streetMeanSpeed(id);
        if (meanSpeed.has_value()) {
          streetFlow << meanSpeed.value() * street->density() << ';';
        } else {
          streetFlow << 0 << ';';
        }
      }
      streetFlow << std::endl;
#endif
#ifdef PRINT_SPEEDS
      streetSpeed << dynamics.time() << ';';
      for (const auto& [id, street] : dynamics.graph().streetSet()) {
        const auto& meanSpeed = dynamics.streetMeanSpeed(id);
        if (meanSpeed.has_value()) {
          streetSpeed << meanSpeed.value() << ';';
        } else {
          streetSpeed << 0 << ';';
        }
      }
      streetSpeed << std::endl;
#endif
    }
    ++progress;
  }
  out.close();
#ifdef PRINT_DENSITIES
  streetDensity.close();
#endif
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
  t.join();
  std::cout << '\n';
  std::cout << "Done." << std::endl;

  return 0;
}
