#include <cassert>
#include <cstdint>

#include "RoadNetwork.hpp"
#include "Node.hpp"
#include "Road.hpp"
#include "Street.hpp"
#include "AdjacencyMatrix.hpp"

#include "doctest.h"

using RoadNetwork = dsf::RoadNetwork;
using AdjacencyMatrix = dsf::AdjacencyMatrix;
using Street = dsf::Street;
using Road = dsf::Road;
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
  SUBCASE("Empty Constructor") {
    RoadNetwork network{};
    network.addEdge<Street>(1, std::make_pair(0, 1));
    CHECK_EQ(network.nEdges(), 1);
    CHECK_EQ(network.nNodes(), 2);
  }
  SUBCASE("AdjacencyMatrix Constructor") {
    AdjacencyMatrix sm;
    sm.insert(0, 1);
    sm.insert(1, 0);
    sm.insert(1, 2);
    sm.insert(2, 3);
    sm.insert(3, 2);
    RoadNetwork graph{sm};
    CHECK_EQ(graph.nNodes(), 4);
    CHECK_EQ(graph.nEdges(), 5);
    CHECK(graph.edge(1, 2));
    CHECK(graph.edge(2, 3));
    CHECK(graph.edge(3, 2));
    CHECK_THROWS_AS(graph.edge(2, 1), std::out_of_range);
  }

  SUBCASE("Construction with addEdge") {
    RoadNetwork graph;
    graph.addEdge<Street>(1, std::make_pair(0, 1));
    graph.addEdge<Street>(2, std::make_pair(1, 2));
    graph.addEdge<Street>(3, std::make_pair(0, 2));
    graph.addEdge<Street>(4, std::make_pair(0, 3));
    graph.addEdge<Street>(5, std::make_pair(2, 3));
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK(graph.edge(0, 1));
    CHECK(graph.edge(1, 2));
    CHECK(graph.edge(0, 2));
    CHECK(graph.edge(0, 3));
    CHECK(graph.edge(2, 3));
  }
  SUBCASE("Construction with addStreets") {
    Street s1(1, std::make_pair(0, 1));
    Street s2(2, std::make_pair(1, 2));
    Street s3(3, std::make_pair(0, 2));
    Street s4(4, std::make_pair(0, 3));
    Street s5(5, std::make_pair(2, 3));
    RoadNetwork graph;
    graph.addStreets(s1, s2, s3, s4, s5);
    CHECK_EQ(graph.nEdges(), 5);
    CHECK_EQ(graph.nNodes(), 4);
    CHECK(graph.edge(0, 1));
    CHECK(graph.edge(1, 2));
    CHECK(graph.edge(0, 2));
    CHECK(graph.edge(0, 3));
    CHECK(graph.edge(2, 3));
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
      CHECK_EQ(graph.nEdges(), 6);
      CHECK_EQ(graph.nNodes(), 4);
      WHEN("We automatically map street lanes") {
        // dsf::Logger::setLogLevel(dsf::log_level_t::DEBUG);
        graph.autoMapStreetLanes();
        // dsf::Logger::setLogLevel(dsf::log_level_t::INFO);
        THEN("The lanes are correctly mapped") {
          CHECK_EQ(graph.edge(0, 1)->laneMapping().size(), 3);
          CHECK_EQ(graph.edge(0, 1)->laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 1)->laneMapping()[1], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 1)->laneMapping()[2], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(1, 0)->laneMapping().size(), 3);
          CHECK_EQ(graph.edge(1, 0)->laneMapping()[0], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(1, 0)->laneMapping()[1], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(1, 0)->laneMapping()[2], dsf::Direction::LEFT);
          CHECK_EQ(graph.edge(0, 2)->laneMapping().size(), 1);
          CHECK_EQ(graph.edge(0, 2)->laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(2, 0)->laneMapping().size(), 1);
          CHECK_EQ(graph.edge(2, 0)->laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 3)->laneMapping().size(), 2);
          CHECK_EQ(graph.edge(0, 3)->laneMapping()[0], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(0, 3)->laneMapping()[1], dsf::Direction::ANY);
          CHECK_EQ(graph.edge(3, 0)->laneMapping().size(), 2);
          CHECK_EQ(graph.edge(3, 0)->laneMapping()[0], dsf::Direction::RIGHT);
          CHECK_EQ(graph.edge(3, 0)->laneMapping()[1], dsf::Direction::STRAIGHT);
        }
      }
    }
  }
  SUBCASE("importMatrix - dsf") {
    // This tests the importMatrix function over .dsf files
    // GIVEN: a graph
    // WHEN: we import a .dsf file
    // THEN: the graph's adjacency matrix is the same as the one in the file
    GIVEN("An empty graph") {
      RoadNetwork graph{};
      WHEN("A matrix in dsf format is imported") {
        graph.importMatrix("./data/matrix.dsf");
        THEN("The graph is correctly built") {
          CHECK(graph.edge(2, 2));
          CHECK(graph.edge(2, 0));
          CHECK(graph.edge(1, 0));
          CHECK(graph.edge(0, 1));
          CHECK_EQ(graph.nNodes(), 3);
          CHECK_EQ(graph.nEdges(), 4);
        }
        THEN("It is correctly exported") { graph.exportMatrix("./data/temp.dsf", true); }
      }
      WHEN("The exported one is imported") {
        graph.importMatrix("./data/temp.dsf");
        THEN("The graph is correctly built") {
          CHECK(graph.edge(2, 2));
          CHECK(graph.edge(2, 0));
          CHECK(graph.edge(1, 0));
          CHECK(graph.edge(0, 1));
          CHECK_EQ(graph.nNodes(), 3);
          CHECK_EQ(graph.nEdges(), 4);
        }
      }
    }
    SUBCASE("Coordinates import/export") {
      GIVEN("A RoadNetwork object with the adj matrix imported") {
        RoadNetwork graph{};
        graph.importMatrix("./data/matrix.dsf");
        WHEN("We import the coordinates in dsf format") {
          graph.importCoordinates("./data/coords.dsf");
          THEN("The coordinates are correctly imported") {
            CHECK_EQ(graph.node(0)->coords(), std::make_pair(0., 0.));
            CHECK_EQ(graph.node(1)->coords(), std::make_pair(1., 0.));
            CHECK_EQ(graph.node(2)->coords(), std::make_pair(2., 0.));
          }
          THEN("The adjacency matrix is correctly built") {
            CHECK(graph.edge(0, 1));
            CHECK(graph.edge(1, 0));
            CHECK(graph.edge(2, 0));
            CHECK(graph.edge(2, 2));
          }
          THEN("The streets angles are correctly computed") {
            CHECK_EQ(graph.edge(1)->angle(), std::numbers::pi / 2);
            CHECK_EQ(graph.edge(3)->angle(), 3 * std::numbers::pi / 2);
            CHECK_EQ(graph.edge(6)->angle(), 3 * std::numbers::pi / 2);
            CHECK_EQ(graph.edge(8)->angle(), 0.);
          }
          THEN("We are able to save nodes and edges in csv format") {
            graph.exportNodes("./data/nodes.csv");
            graph.exportEdges("./data/edges.csv");
          }
        }
        WHEN("We import the coordinates in csv format") {
          graph.importCoordinates("./data/nodes.csv");
          THEN("The coordinates are correctly imported") {
            CHECK_EQ(graph.node(0)->coords(), std::make_pair(0., 0.));
            CHECK_EQ(graph.node(1)->coords(), std::make_pair(1., 0.));
            CHECK_EQ(graph.node(2)->coords(), std::make_pair(2., 0.));
          }
        }
      }
    }
    SUBCASE("importMatrix - raw matrix") {
      RoadNetwork graph{};
      graph.importMatrix("./data/rawMatrix.txt", false);
      CHECK_EQ(graph.nNodes(), 3);
      CHECK(graph.edge(0, 1));
      CHECK(graph.edge(1, 0));
      CHECK(graph.edge(1, 2));
      CHECK(graph.edge(2, 1));
      CHECK_EQ(graph.nNodes(), 3);
      CHECK_EQ(graph.nEdges(), 4);
      CHECK_EQ(graph.edge(1)->length(), 500);
      CHECK_EQ(graph.edge(3)->length(), 200);
      CHECK_EQ(graph.edge(5)->length(), 1);
      CHECK_EQ(graph.edge(7)->length(), 3);
    }
    SUBCASE("importMatrix - EXCEPTIONS") {
      // This tests the importMatrix throws an exception when the file has not the correct format or is not found
      // GIVEN: a graph
      // WHEN: we import a file with a wrong format
      // THEN: an exception is thrown
      RoadNetwork graph{};
      CHECK_THROWS(graph.importMatrix("./data/matrix.nogood"));
      CHECK_THROWS(graph.importMatrix("./data/not_found.dsf"));
    }
    SUBCASE("importNodes and importEdges") {
      GIVEN("A graph object") {
        RoadNetwork graph;
        WHEN("We import nodes and edges from OSM") {
          graph.importNodes("./data/postua_nodes.csv");
          graph.importEdges("./data/postua_edges.csv");
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
            CHECK_EQ(graph.nCoils(), 0);
            CHECK_EQ(graph.nIntersections(), 24);
            CHECK_EQ(graph.nRoundabouts(), 1);
            CHECK_EQ(graph.nTrafficLights(), 0);
          }
          RoadNetwork graph2;
          graph2.importEdges("./data/postua_edges.geojson");
          THEN("Sizes are correct also with geojson") {
            CHECK_EQ(graph2.nEdges(), graph.nEdges());
            CHECK_EQ(graph2.nNodes(), graph.nNodes());
            for (auto const& [edgeId, pEdge] : graph2.edges()) {
              CHECK_EQ(*pEdge, *graph.edge(edgeId));
            }
          }
        }
        WHEN("We import many nodes and edges from OSM") {
          graph.importNodes("./data/forlì_nodes.csv");
          graph.importEdges("./data/forlì_edges.csv");
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
            CHECK_EQ(graph.nCoils(), 0);
            CHECK_EQ(graph.nIntersections(), 3101);
            CHECK_EQ(graph.nRoundabouts(), 2);
            CHECK_EQ(graph.nTrafficLights(), 15);
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
        graph2.addNDefaultNodes(5);
        graph2.makeTrafficLight(1, 120);
        graph2.addStreets(s1, s2, s3, s4);
        for (auto const& [streetId, pStreet] : graph2.edges()) {
          pStreet->setCapacity(2 * pStreet->nLanes());
        }
        WHEN("We auto-init Traffic Lights") {
          graph2.initTrafficLights();
          THEN("Parameters are correctly set") {
            auto& tl{graph2.node<dsf::TrafficLight>(1)};
            CHECK_EQ(tl.cycleTime(), 120);
            auto const& cycles{tl.cycles()};
            CHECK_EQ(cycles.size(), 4);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 53);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::LEFT).greenTime(), 26);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::RIGHTANDSTRAIGHT).phase(), 0);
            CHECK_EQ(cycles.at(1).at(dsf::Direction::LEFT).phase(), 53);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::RIGHTANDSTRAIGHT).greenTime(), 53);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::LEFT).greenTime(), 26);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::RIGHTANDSTRAIGHT).phase(), 0);
            CHECK_EQ(cycles.at(11).at(dsf::Direction::LEFT).phase(), 53);
            CHECK_EQ(cycles.at(16).at(dsf::Direction::ANY).greenTime(), 40);
            CHECK_EQ(cycles.at(16).at(dsf::Direction::ANY).phase(), 80);
            CHECK_EQ(cycles.at(21).at(dsf::Direction::ANY).greenTime(), 40);
            CHECK_EQ(cycles.at(21).at(dsf::Direction::ANY).phase(), 80);
          }
        }
      }
    }
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addStreet(Street{1, std::make_pair(0, 1)});
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
      WHEN("We make node 0 a roundabout") {
        graph.makeRoundabout(0);
        THEN("The node 0 is a roundabout") { CHECK(graph.node(0)->isRoundabout()); }
      }
    }
  }
  SUBCASE("make spire street") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addEdge(Street{0, std::make_pair(0, 1)});
      WHEN("We make the street a spire street") {
        graph.makeSpireStreet(0);
        THEN("The street is a spire street") { CHECK(graph.edge(0)->isSpire()); }
      }
    }
  }
  SUBCASE("street and oppositeStreet") {
    GIVEN("A RoadNetwork object with two streets") {
      RoadNetwork graph{};
      Street street{1, std::make_pair(0, 1), 1.};
      Street opposite{2, std::make_pair(1, 0), 1.};
      graph.addNDefaultNodes(2);
      graph.addStreets(street, opposite);
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
        auto result = graph.street(1, 0);
        THEN("The opposite street is found and has correct values") {
          CHECK(result);
          const auto& road = *result;
          CHECK_EQ(road->id(), 2);
          CHECK_EQ(road->length(), 1.);
          CHECK_EQ(road->capacity(), 1);
        }
      }
      WHEN("We search for a not existing street") {
        THEN("The street is not found") { CHECK_FALSE(graph.street(1, 2)); }
      }
      WHEN("We search for the opposite of a not existing street") {
        THEN("It throws an exception") { CHECK_FALSE(graph.street(2, 1)); }
      }
    }
  }

  SUBCASE("adjustNodeCapacities and normalizeStreetCapacities") {
    GIVEN("A graph composed of three streets with a different lane number") {
      Street s1(0, std::make_pair(0, 1), 10., 30., 1);
      Street s2(1, std::make_pair(1, 2), 40., 30., 2);
      Street s3(2, std::make_pair(3, 1), 75., 30., 3);
      Street s4(3, std::make_pair(1, 4), 55., 30., 1);
      RoadNetwork graph{};
      graph.addStreets(s1, s2, s3, s4);
      WHEN("We adjust node capacities") {
        graph.adjustNodeCapacities();
        THEN("The node capacities are correct") {
          CHECK_EQ(graph.node(0)->capacity(), 1);
          CHECK_EQ(graph.node(1)->capacity(), 4);
          CHECK_EQ(graph.node(2)->capacity(), 2);
          CHECK_EQ(graph.node(3)->capacity(), 3);
          CHECK_EQ(graph.node(4)->capacity(), 1);
        }
        THEN("The transport capacities are correct") {
          CHECK_EQ(graph.node(0)->transportCapacity(), 1);
          CHECK_EQ(graph.node(1)->transportCapacity(), 3);
          CHECK_EQ(graph.node(2)->transportCapacity(), 1);
          CHECK_EQ(graph.node(3)->transportCapacity(), 3);
          CHECK_EQ(graph.node(4)->transportCapacity(), 1);
        }
      }
      WHEN("We normalize street capacities") {
        // graph.normalizeStreetCapacities();
        THEN("The street capacities are correct") {
          CHECK_EQ(graph.edge(0, 1)->capacity(), 2);
          CHECK_EQ(graph.edge(1, 2)->capacity(), 16);
          CHECK_EQ(graph.edge(3, 1)->capacity(), 45);
          CHECK_EQ(graph.edge(1, 4)->capacity(), 11);
        }
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

    auto const& pathMap =
        graph.allPathsTo(2, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 0);
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("Case 2") {
    Street s1(0, std::make_pair(0, 1), 1.);
    Street s2(1, std::make_pair(1, 2), 1.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);

    auto const& pathMap =
        graph.allPathsTo(2, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
  }

  SUBCASE("Case 3") {
    Street s1(0, std::make_pair(0, 1), 5.);
    Street s2(1, std::make_pair(1, 2), 4.);
    Street s3(2, std::make_pair(0, 2), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);

    auto const& pathMap =
        graph.allPathsTo(2, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 2);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);
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

    auto const& pathMap =
        graph.allPathsTo(4, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(4));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 4);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 0);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 1);
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

    auto const& pathMap =
        graph.allPathsTo(6, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(6));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 4);
    CHECK_EQ(pathMap.at(4).size(), 1);
    CHECK_EQ(pathMap.at(4)[0], 6);
    CHECK_EQ(pathMap.at(5).size(), 1);
    CHECK_EQ(pathMap.at(5)[0], 6);
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

    auto const& pathMap =
        graph.allPathsTo(4, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_FALSE(pathMap.contains(4));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 2);
    CHECK_EQ(pathMap.at(1).size(), 2);
    CHECK_EQ(pathMap.at(1)[0], 3);
    CHECK_EQ(pathMap.at(1)[1], 2);
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 5);
    CHECK_EQ(pathMap.at(3).size(), 1);
    CHECK_EQ(pathMap.at(3)[0], 4);
    CHECK_EQ(pathMap.at(5).size(), 1);
    CHECK_EQ(pathMap.at(5)[0], 4);
  }

  SUBCASE("Case 7") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);

    auto const& pathMap =
        graph.allPathsTo(1, [](auto const& pEdge) { return pEdge->length(); });
    CHECK(pathMap.empty());
  }

  SUBCASE("Case 8") {
    Street s1(0, std::make_pair(1, 2), 1.);
    Street s2(1, std::make_pair(0, 2), 6.);
    Street s3(2, std::make_pair(2, 0), 6.);
    RoadNetwork graph{};
    graph.addStreets(s1, s2, s3);

    CHECK_THROWS_AS(
        graph.allPathsTo(3, [](auto const& pEdge) { return pEdge->length(); }),
        std::out_of_range);
  }

  SUBCASE("Case 9 - Equal Lengths") {
    RoadNetwork graph{};
    graph.importMatrix("./data/matrix.dat", false);
    // check correct import
    CHECK_EQ(graph.nNodes(), 120);
    CHECK_EQ(graph.nEdges(), 436);
    // check that the path exists
    CHECK(graph.edge(46, 58));
    CHECK(graph.edge(58, 70));
    CHECK(graph.edge(70, 82));
    CHECK(graph.edge(82, 94));
    CHECK(graph.edge(94, 106));
    CHECK(graph.edge(106, 118));

    auto const& path =
        graph.allPathsTo(118, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_EQ(path.size(), 119);
    CHECK_FALSE(path.contains(118));
  }
}
