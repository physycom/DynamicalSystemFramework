
#include <cassert>
#include <cstdint>

#include "RoadNetwork.hpp"
#include "Node.hpp"
#include "Road.hpp"
#include "Street.hpp"
#include "AdjacencyMatrix.hpp"

#include "doctest.h"

using RoadNetwork = dsm::RoadNetwork;
using AdjacencyMatrix = dsm::AdjacencyMatrix;
using Street = dsm::Street;
using Road = dsm::Road;
using Path = std::vector<uint>;

template <typename T1, typename T2>
bool checkPath(const std::vector<T1>& path1, const std::vector<T2>& path2) {
  const size_t length{path1.size()};
  assert(length == path2.size());

  bool equal{true};
  for (size_t i{}; i < length; ++i) {
    if (path1[i] != path2[i]) {
      equal = false;
    }
  }

  return equal;
}

TEST_CASE("RoadNetwork") {
  Road::setMeanVehicleLength(5.);
  SUBCASE("Constructor_1") {
    RoadNetwork graph{};
    graph.addEdge<Street>(1, std::make_pair(0, 1));
    graph.buildAdj();
    CHECK_EQ(graph.nEdges(), 1);
    CHECK_EQ(graph.nNodes(), 2);
    CHECK_EQ(graph.adjacencyMatrix().size(), graph.nEdges());
  }

  SUBCASE("Constructor_2") {
    AdjacencyMatrix sm;
    sm.insert(0, 1);
    sm.insert(1, 0);
    sm.insert(1, 2);
    sm.insert(2, 3);
    sm.insert(3, 2);
    RoadNetwork graph{sm};
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.adjacencyMatrix().size(), graph.nEdges());
    CHECK(graph.adjacencyMatrix().contains(1, 2));
    CHECK(graph.adjacencyMatrix().contains(2, 3));
    CHECK(graph.adjacencyMatrix().contains(3, 2));
    CHECK_FALSE(graph.adjacencyMatrix().contains(2, 1));
  }

  SUBCASE("Construction with addStreet") {
    RoadNetwork graph;
    graph.addEdge<Street>(1, std::make_pair(0, 1));
    graph.addEdge<Street>(2, std::make_pair(1, 2));
    graph.addEdge<Street>(3, std::make_pair(0, 2));
    graph.addEdge<Street>(4, std::make_pair(0, 3));
    graph.addEdge<Street>(5, std::make_pair(2, 3));
    graph.buildAdj();

    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_EQ(graph.adjacencyMatrix().size(), 5);
    CHECK(graph.adjacencyMatrix().contains(0, 1));
    CHECK(graph.adjacencyMatrix().contains(1, 2));
    CHECK(graph.adjacencyMatrix().contains(0, 2));
    CHECK_FALSE(graph.adjacencyMatrix().contains(1, 3));
  }

  SUBCASE("Construction with addStreets") {
    Street s1(1, std::make_pair(0, 1));
    Street s2(2, std::make_pair(1, 2));
    Street s3(3, std::make_pair(0, 2));
    Street s4(4, std::make_pair(0, 3));
    Street s5(5, std::make_pair(2, 3));
    RoadNetwork graph;
    graph.addStreets(s1, s2, s3, s4, s5);
    graph.buildAdj();

    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_EQ(graph.adjacencyMatrix().size(), graph.nEdges());
    CHECK(graph.adjacencyMatrix().contains(0, 1));
    CHECK(graph.adjacencyMatrix().contains(1, 2));
    CHECK(graph.adjacencyMatrix().contains(0, 2));
    CHECK_FALSE(graph.adjacencyMatrix().contains(1, 3));
  }
  SUBCASE("automatically do things") {
    GIVEN("A Graph object") {
      RoadNetwork graph{};
      graph.addNode(0, std::make_pair(0., 0.));
      graph.addNode(1, std::make_pair(0., -1.));
      graph.addNode(2, std::make_pair(-1., 0.));
      graph.addNode(3, std::make_pair(1., 1.));
      graph.addEdge<Street>(1, std::make_pair(0, 1), 1., 50 / 3.6, 3);
      graph.addEdge<Street>(4, std::make_pair(1, 0), 1., 50 / 3.6, 3);
      graph.addEdge<Street>(2, std::make_pair(0, 2), 1., 30 / 3.6);
      graph.addEdge<Street>(8, std::make_pair(2, 0), 1., 30 / 3.6);
      graph.addEdge<Street>(3, std::make_pair(0, 3), 1., 30 / 3.6, 2);
      graph.addEdge<Street>(12, std::make_pair(3, 0), 1., 30 / 3.6, 2);
      graph.buildAdj();
      CHECK_EQ(graph.nEdges(), 6);
      CHECK_EQ(graph.nNodes(), 4);
      WHEN("We automatically map street lanes") {
        // dsm::Logger::setLogLevel(dsm::log_level_t::DEBUG);
        graph.autoSetStreetPriorities();
        graph.autoMapStreetLanes();
        // dsm::Logger::setLogLevel(dsm::log_level_t::INFO);
        THEN("The lanes are correctly mapped") {
          CHECK_EQ(graph.edge(1)->laneMapping().size(), 3);
          CHECK_EQ(graph.edge(1)->laneMapping()[0], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(1)->laneMapping()[1], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(1)->laneMapping()[2], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(4)->laneMapping().size(), 3);
          CHECK_EQ(graph.edge(4)->laneMapping()[0], dsm::Direction::RIGHT);
          CHECK_EQ(graph.edge(4)->laneMapping()[1], dsm::Direction::RIGHT);
          CHECK_EQ(graph.edge(4)->laneMapping()[2], dsm::Direction::LEFT);
          CHECK_EQ(graph.edge(2)->laneMapping().size(), 1);
          CHECK_EQ(graph.edge(2)->laneMapping()[0], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(8)->laneMapping().size(), 1);
          CHECK_EQ(graph.edge(8)->laneMapping()[0], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(3)->laneMapping().size(), 2);
          CHECK_EQ(graph.edge(3)->laneMapping()[0], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(3)->laneMapping()[1], dsm::Direction::ANY);
          CHECK_EQ(graph.edge(12)->laneMapping().size(), 2);
          CHECK_EQ(graph.edge(12)->laneMapping()[0], dsm::Direction::RIGHT);
          CHECK_EQ(graph.edge(12)->laneMapping()[1], dsm::Direction::STRAIGHT);
        }
      }
      WHEN("We automatically set street priorities") {
        graph.autoSetStreetPriorities();
        THEN("The priorities are correctly set") {
          // Dios, qué pereza
          CHECK_FALSE(graph.edge(1)->hasPriority());
          CHECK_FALSE(graph.edge(2)->hasPriority());
          CHECK_FALSE(graph.edge(3)->hasPriority());
          CHECK(graph.edge(4)->hasPriority());
          CHECK_FALSE(graph.edge(8)->hasPriority());
          CHECK_FALSE(graph.edge(12)->hasPriority());
        }
      }
    }
  }
  SUBCASE("importMatrix - dsm") {
    // This tests the importMatrix function over .dsm files
    // GIVEN: a graph
    // WHEN: we import a .dsm file
    // THEN: the graph's adjacency matrix is the same as the one in the file
    GIVEN("An empty graph") {
      RoadNetwork graph{};
      WHEN("A matrix in dsm format is imported") {
        graph.importMatrix("./data/matrix.dsm");
        THEN("The graph is correctly built") {
          CHECK_EQ(graph.adjacencyMatrix().n(), 3);
          CHECK(graph.adjacencyMatrix().operator()(2, 2));
          CHECK(graph.adjacencyMatrix().operator()(2, 0));
          CHECK(graph.adjacencyMatrix().operator()(1, 0));
          CHECK(graph.adjacencyMatrix().operator()(0, 1));
          CHECK_EQ(graph.nNodes(), 3);
          CHECK_EQ(graph.nEdges(), 4);
        }
        THEN("It is correctly exported") { graph.exportMatrix("./data/temp.dsm", true); }
      }
      WHEN("The exported one is imported") {
        graph.importMatrix("./data/temp.dsm");
        THEN("The graph is correctly built") {
          CHECK_EQ(graph.adjacencyMatrix().n(), 3);
          CHECK(graph.adjacencyMatrix().operator()(2, 2));
          CHECK(graph.adjacencyMatrix().operator()(2, 0));
          CHECK(graph.adjacencyMatrix().operator()(1, 0));
          CHECK(graph.adjacencyMatrix().operator()(0, 1));
          CHECK_EQ(graph.nNodes(), 3);
          CHECK_EQ(graph.nEdges(), 4);
        }
      }
    }
  }
  SUBCASE("Coordinates import/export") {
    GIVEN("A RoadNetwork object with the adj matrix imported") {
      RoadNetwork graph{};
      graph.importMatrix("./data/matrix.dsm");
      auto const& nodes = graph.nodes();
      WHEN("We import the coordinates in dsm format") {
        graph.importCoordinates("./data/coords.dsm");
        THEN("The coordinates are correctly imported") {
          CHECK_EQ(nodes.at(0)->coords(), std::make_pair(0., 0.));
          CHECK_EQ(nodes.at(1)->coords(), std::make_pair(1., 0.));
          CHECK_EQ(nodes.at(2)->coords(), std::make_pair(2., 0.));
        }
        graph.buildAdj();
        auto const& adj{graph.adjacencyMatrix()};
        THEN("The adjacency matrix is correctly built") {
          CHECK(adj(0, 1));
          CHECK(adj(1, 0));
          CHECK(adj(2, 0));
          CHECK(adj(2, 2));
        }
        auto const& streets{graph.edges()};
        THEN("The streets angles are correctly computed") {
          CHECK_EQ(streets.at(1)->angle(), std::numbers::pi / 2);
          CHECK_EQ(streets.at(3)->angle(), 3 * std::numbers::pi / 2);
          CHECK_EQ(streets.at(6)->angle(), 3 * std::numbers::pi / 2);
          CHECK_EQ(streets.at(8)->angle(), 0.);
        }
        THEN("We are able to save nodes and edges in csv format") {
          graph.exportNodes("./data/nodes.csv");
          graph.exportEdges("./data/edges.csv");
        }
      }
      WHEN("We import the coordinates in csv format") {
        graph.importCoordinates("./data/nodes.csv");
        THEN("The coordinates are correctly imported") {
          CHECK_EQ(nodes.at(0)->coords(), std::make_pair(0., 0.));
          CHECK_EQ(nodes.at(1)->coords(), std::make_pair(1., 0.));
          CHECK_EQ(nodes.at(2)->coords(), std::make_pair(2., 0.));
        }
      }
    }
  }
  SUBCASE("importMatrix - raw matrix") {
    RoadNetwork graph{};
    graph.importMatrix("./data/rawMatrix.txt", false);
    CHECK_EQ(graph.adjacencyMatrix().n(), 3);
    CHECK(graph.adjacencyMatrix().operator()(0, 1));
    CHECK(graph.adjacencyMatrix().operator()(1, 0));
    CHECK(graph.adjacencyMatrix().operator()(1, 2));
    CHECK(graph.adjacencyMatrix().operator()(2, 1));
    CHECK_EQ(graph.nNodes(), 3);
    CHECK_EQ(graph.nEdges(), 4);
    CHECK_EQ(graph.edges().at(1)->length(), 500);
    CHECK_EQ(graph.edges().at(3)->length(), 200);
    CHECK_EQ(graph.edges().at(5)->length(), 1);
    CHECK_EQ(graph.edges().at(7)->length(), 3);
  }
  SUBCASE("importMatrix - EXCEPTIONS") {
    // This tests the importMatrix throws an exception when the file has not the correct format or is not found
    // GIVEN: a graph
    // WHEN: we import a file with a wrong format
    // THEN: an exception is thrown
    RoadNetwork graph{};
    CHECK_THROWS(graph.importMatrix("./data/matrix.nogood"));
    CHECK_THROWS(graph.importMatrix("./data/not_found.dsm"));
  }
  SUBCASE("importOSMNodes and importOSMEdges") {
    GIVEN("A graph object") {
      RoadNetwork graph{};
      WHEN("We import nodes and edges from OSM") {
        graph.importOSMNodes("./data/postua_nodes.csv");
        graph.importOSMEdges("./data/postua_edges.csv");
        std::ifstream fNodes{"./data/postua_nodes.csv"};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{"./data/postua_edges.csv"};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
        }
        THEN("We are able to build the adjacency matrix") {
          graph.buildAdj();
          CHECK_EQ(graph.adjacencyMatrix().size(), nEdges);
        }
      }
      WHEN("We import many nodes and edges from OSM") {
        graph.importOSMNodes("./data/forlì_nodes.csv");
        graph.importOSMEdges("./data/forlì_edges.csv");
        std::ifstream fNodes{"./data/forlì_nodes.csv"};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{"./data/forlì_edges.csv"};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
        }
        THEN("We are able to build the adjacency matrix") {
          graph.buildAdj();
          CHECK_EQ(graph.adjacencyMatrix().size(), nEdges);
        }
      }
    }
  }
  SUBCASE("street") {
    /// GIVEN: a graph
    /// WHEN: we add a street
    /// THEN: the street is added
    RoadNetwork graph{};
    graph.addEdge<Street>(1, std::make_pair(0, 1), 1.);
    auto result = graph.street(0, 1);
    CHECK(result);
    const auto& street2 = *result;
    CHECK_EQ(street2->id(), 1);
    CHECK_EQ(street2->length(), 1.);
    CHECK_EQ(street2->capacity(), 1);
    CHECK_FALSE(graph.street(1, 0));
  }
  SUBCASE("make trafficlight") {
    GIVEN("A graph with a traffic light with no parameters") {
      Street s1{1, std::make_pair(0, 1), 30., 15., 3};
      Street s2{11, std::make_pair(2, 1), 30., 15., 3};
      Street s3{16, std::make_pair(3, 1), 30., 15., 1};
      Street s4{21, std::make_pair(4, 1), 30., 15., 2};
      RoadNetwork graph2;
      graph2.addNode<dsm::TrafficLight>(1, 120);
      graph2.addStreets(s1, s2, s3, s4);
      for (auto const& pair : graph2.edges()) {
        pair.second->setCapacity(2 * pair.second->nLanes());
      }
      graph2.buildAdj();
      WHEN("We auto-init Traffic Lights") {
        graph2.initTrafficLights();
        THEN("Parameters are correctly set") {
          auto& tl{graph2.node<dsm::TrafficLight>(1)};
          CHECK_EQ(tl.cycleTime(), 120);
          auto const& cycles{tl.cycles()};
          CHECK_EQ(cycles.size(), 4);
          CHECK_EQ(cycles.at(1)[0].greenTime(), 53);
          CHECK_EQ(cycles.at(1)[1].greenTime(), 53);
          CHECK_EQ(cycles.at(1)[2].greenTime(), 26);
          CHECK_EQ(cycles.at(1)[0].phase(), 0);
          CHECK_EQ(cycles.at(1)[1].phase(), 0);
          CHECK_EQ(cycles.at(1)[2].phase(), 53);
          CHECK_EQ(cycles.at(11)[0].greenTime(), 53);
          CHECK_EQ(cycles.at(11)[1].greenTime(), 53);
          CHECK_EQ(cycles.at(11)[2].greenTime(), 26);
          CHECK_EQ(cycles.at(11)[0].phase(), 0);
          CHECK_EQ(cycles.at(11)[1].phase(), 0);
          CHECK_EQ(cycles.at(11)[2].phase(), 53);
          CHECK_EQ(cycles.at(16)[0].greenTime(), 40);
          CHECK_EQ(cycles.at(16)[1].greenTime(), 40);
          CHECK_EQ(cycles.at(16)[2].greenTime(), 40);
          CHECK_EQ(cycles.at(16)[0].phase(), 80);
          CHECK_EQ(cycles.at(16)[1].phase(), 80);
          CHECK_EQ(cycles.at(16)[2].phase(), 80);
          CHECK_EQ(cycles.at(21)[0].greenTime(), 40);
          CHECK_EQ(cycles.at(21)[1].greenTime(), 40);
          CHECK_EQ(cycles.at(21)[2].greenTime(), 40);
          CHECK_EQ(cycles.at(21)[0].phase(), 80);
          CHECK_EQ(cycles.at(21)[1].phase(), 80);
          CHECK_EQ(cycles.at(21)[2].phase(), 80);
        }
      }
    }
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{1, std::make_pair(0, 1)});
      graph.buildAdj();
      WHEN("We make node 0 a traffic light") {
        auto& tl = graph.makeTrafficLight(0, 60);
        THEN("The node 0 is a traffic light") { CHECK(graph.node(0)->isTrafficLight()); }
        THEN("The traffic light has the correct parameters") {
          CHECK_EQ(tl.id(), 0);
          CHECK_EQ(tl.cycleTime(), 60);
        }
      }
    }
  }
  SUBCASE("make roundabout") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{1, std::make_pair(0, 1)});
      graph.buildAdj();
      WHEN("We make node 0 a roundabout") {
        graph.makeRoundabout(0);
        THEN("The node 0 is a roundabout") { CHECK(graph.node(0)->isRoundabout()); }
      }
    }
  }
  SUBCASE("make spire street") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{0, std::make_pair(0, 1)});
      graph.buildAdj();
      WHEN("We make the street a spire street") {
        graph.makeSpireStreet(1);
        THEN("The street is a spire street") { CHECK(graph.edge(1)->isSpire()); }
      }
    }
  }
}

TEST_CASE("Dijkstra") {
  SUBCASE("Case 1") {
    Street s1{0, std::make_pair(0, 1), 3.};
    Street s2{1, std::make_pair(1, 2), 2.};
    Street s3{2, std::make_pair(2, 3), 4.};
    Street s4{3, std::make_pair(3, 0), 5.};
    Street s5{4, std::make_pair(0, 2), 6.};
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3, s4, s5);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 1);
    Path correctPath{0, 1};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 3.);
    result = graph.shortestPath(0, 2);
    correctPath = Path{0, 1, 2};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 3);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 5.);
  }

  SUBCASE("Case 2") {
    Street s1(0, std::make_pair(0, 1), 1.);
    Street s2(1, std::make_pair(1, 2), 1.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 2);
    Path correctPath{0, 1, 2};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 3);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 2.);
  }

  SUBCASE("Case 3") {
    Street s1(0, std::make_pair(0, 1), 5.);
    Street s2(1, std::make_pair(1, 2), 4.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 2);
    Path correctPath{0, 2};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 6.);
  }

  SUBCASE("Case 4") {
    Street s1(0, std::make_pair(0, 1), 3.);
    Street s2(1, std::make_pair(0, 2), 1.);
    Street s3(2, std::make_pair(1, 2), 7.);
    Street s4(3, std::make_pair(2, 3), 2.);
    Street s5(4, std::make_pair(1, 4), 1.);
    Street s6(5, std::make_pair(1, 3), 5.);
    Street s7(6, std::make_pair(3, 4), 7.);
    Street s8(7, std::make_pair(1, 0), 3.);
    Street s9(8, std::make_pair(2, 0), 1.);
    Street s10(9, std::make_pair(2, 1), 7.);
    Street s11(10, std::make_pair(3, 2), 2.);
    Street s12(11, std::make_pair(4, 1), 1.);
    Street s13(12, std::make_pair(3, 1), 5.);
    Street s14(13, std::make_pair(4, 3), 7.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14);
    graph.buildAdj();
    auto result = graph.shortestPath(2, 4);
    Path correctPath{2, 0, 1, 4};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 4);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 5.);
    result = graph.shortestPath(2, 0);
    correctPath = Path{2, 0};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 1.);
    result = graph.shortestPath(2, 1);
    correctPath = Path{2, 0, 1};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 3);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 4.);
    result = graph.shortestPath(2, 3);
    correctPath = Path{2, 3};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 2.);
  }

  SUBCASE("Case 5") {
    Street s1(0, std::make_pair(0, 1), 2.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(1, 3), 5.);
    Street s4(3, std::make_pair(2, 3), 8.);
    Street s5(4, std::make_pair(3, 5), 15.);
    Street s6(5, std::make_pair(3, 4), 10.);
    Street s7(6, std::make_pair(4, 5), 6.);
    Street s8(7, std::make_pair(4, 6), 2.);
    Street s9(8, std::make_pair(5, 6), 6.);
    Street s10(9, std::make_pair(1, 0), 2.);
    Street s11(10, std::make_pair(2, 0), 6.);
    Street s12(11, std::make_pair(3, 1), 5.);
    Street s13(12, std::make_pair(3, 2), 8.);
    Street s14(13, std::make_pair(5, 3), 15.);
    Street s15(14, std::make_pair(4, 3), 10.);
    Street s16(15, std::make_pair(5, 4), 6.);
    Street s17(16, std::make_pair(6, 4), 2.);
    Street s18(17, std::make_pair(6, 5), 6.);
    RoadNetwork graph{};
    graph.addStreets(
        s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 1);
    Path correctPath{0, 1};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 2.);

    result = graph.shortestPath(0, 2);
    correctPath = Path{0, 2};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 2);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 6.);

    result = graph.shortestPath(0, 3);
    correctPath = Path{0, 1, 3};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 3);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 7.);

    result = graph.shortestPath(0, 4);
    correctPath = Path{0, 1, 3, 4};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 4);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 17.);

    result = graph.shortestPath(0, 5);
    correctPath = Path{0, 1, 3, 5};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 4);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 22.);

    result = graph.shortestPath(0, 6);
    correctPath = Path{0, 1, 3, 4, 6};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 5);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 19.);
  }

  SUBCASE("Case 6") {
    Street s1(0, std::make_pair(0, 1), 7.);
    Street s2(1, std::make_pair(0, 2), 9.);
    Street s3(2, std::make_pair(0, 5), 14.);
    Street s4(3, std::make_pair(1, 3), 15.);
    Street s5(4, std::make_pair(1, 2), 10.);
    Street s6(5, std::make_pair(2, 3), 11.);
    Street s7(6, std::make_pair(2, 5), 2.);
    Street s8(7, std::make_pair(3, 4), 6.);
    Street s9(8, std::make_pair(5, 4), 9.);
    Street s10(9, std::make_pair(1, 0), 7.);
    Street s11(10, std::make_pair(2, 0), 9.);
    Street s12(11, std::make_pair(5, 0), 14.);
    Street s13(12, std::make_pair(3, 1), 15.);
    Street s14(13, std::make_pair(2, 1), 10.);
    Street s15(14, std::make_pair(3, 2), 11.);
    Street s16(15, std::make_pair(5, 2), 2.);
    Street s17(16, std::make_pair(4, 3), 6.);
    Street s18(17, std::make_pair(4, 5), 9.);
    RoadNetwork graph{};
    graph.addStreets(
        s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 4);
    Path correctPath{0, 2, 5, 4};
    CHECK(result.has_value());
    CHECK_EQ(result.value().path().size(), 4);
    CHECK(checkPath(result.value().path(), correctPath));
    CHECK_EQ(result.value().distance(), 20.);
  }

  SUBCASE("Case 7") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);
    graph.buildAdj();
    auto result = graph.shortestPath(0, 1);
    CHECK_FALSE(result.has_value());
  }

  SUBCASE("Case 8") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);
    graph.buildAdj();
    auto result = graph.shortestPath(3, 1);
    CHECK_FALSE(result.has_value());
  }

  SUBCASE("Case 9") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);
    graph.buildAdj();
    auto result = graph.shortestPath(1, 3);
    CHECK_FALSE(result.has_value());
  }

  SUBCASE("street and oppositeStreet") {
    GIVEN("A RoadNetwork object with two streets") {
      RoadNetwork graph{};
      Street street{1, std::make_pair(0, 1), 1.};
      Street opposite{2, std::make_pair(1, 0), 1.};
      graph.addStreets(street, opposite);
      graph.buildAdj();
      WHEN("We search for a street") {
        auto result = graph.street(0, 1);
        THEN("The street is found and has correct values") {
          CHECK(result);
          const auto& road = *result;
          CHECK_EQ(road->id(), 1);
          CHECK_EQ(road->length(), 1.);
          CHECK_EQ(road->capacity(), 1);
        }
      }
      WHEN("We search for the opposite street") {
        auto result = graph.oppositeStreet(1);
        THEN("The opposite street is found and has correct values") {
          CHECK(result);
          const auto& road = *result;
          CHECK_EQ(road->id(), 2);
          CHECK_EQ(road->length(), 1.);
          CHECK_EQ(road->capacity(), 1);
        }
      }
      WHEN("We search for a not existing street") {
        auto result = graph.street(1, 2);
        THEN("The street is not found") { CHECK_FALSE(result); }
      }
      WHEN("We search for the opposite of a not existing street") {
        THEN("It throws an exception") {
          CHECK_THROWS_AS(graph.oppositeStreet(3), std::invalid_argument);
        }
      }
    }
  }

  SUBCASE("equal length") {
    RoadNetwork graph{};
    graph.importMatrix("./data/matrix.dat", false);
    // check correct import
    CHECK_EQ(graph.adjacencyMatrix().n(), 120);
    CHECK_EQ(graph.adjacencyMatrix().size(), 436);
    // check that the path exists
    CHECK(graph.adjacencyMatrix().operator()(46, 58));
    CHECK(graph.adjacencyMatrix().operator()(58, 70));
    CHECK(graph.adjacencyMatrix().operator()(70, 82));
    CHECK(graph.adjacencyMatrix().operator()(82, 94));
    CHECK(graph.adjacencyMatrix().operator()(94, 106));
    CHECK(graph.adjacencyMatrix().operator()(106, 118));

    auto result = graph.shortestPath(46, 118);
    CHECK(result.has_value());
  }
  SUBCASE("adjustNodeCapacities and normalizeStreetCapacities") {
    GIVEN("A graph composed of three streets with a different lane number") {
      Street s1(0, std::make_pair(0, 1), 10., 30., 1);
      Street s2(1, std::make_pair(1, 2), 40., 30., 2);
      Street s3(2, std::make_pair(3, 1), 75., 30., 3);
      Street s4(3, std::make_pair(1, 4), 55., 30., 1);
      RoadNetwork graph{};
      graph.addStreets(s1, s2, s3, s4);
      graph.buildAdj();
      WHEN("We adjust node capacities") {
        graph.adjustNodeCapacities();
        auto const& nodes = graph.nodes();
        THEN("The node capacities are correct") {
          CHECK_EQ(nodes.at(0)->capacity(), 1);
          CHECK_EQ(nodes.at(1)->capacity(), 4);
          CHECK_EQ(nodes.at(2)->capacity(), 2);
          CHECK_EQ(nodes.at(3)->capacity(), 3);
          CHECK_EQ(nodes.at(4)->capacity(), 1);
        }
        THEN("The transport capacities are correct") {
          CHECK_EQ(nodes.at(0)->transportCapacity(), 1);
          CHECK_EQ(nodes.at(1)->transportCapacity(), 3);
          CHECK_EQ(nodes.at(2)->transportCapacity(), 1);
          CHECK_EQ(nodes.at(3)->transportCapacity(), 3);
          CHECK_EQ(nodes.at(4)->transportCapacity(), 1);
        }
      }
      WHEN("We normalize street capacities") {
        // graph.normalizeStreetCapacities();
        auto const& streets = graph.edges();
        THEN("The street capacities are correct") {
          CHECK_EQ(streets.at(1)->capacity(), 2);
          CHECK_EQ(streets.at(7)->capacity(), 16);
          CHECK_EQ(streets.at(16)->capacity(), 45);
          CHECK_EQ(streets.at(9)->capacity(), 11);
        }
      }
    }
  }
}
