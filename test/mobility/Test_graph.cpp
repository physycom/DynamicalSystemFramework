#include "dsf/mobility/RoadNetwork.hpp"
#include "dsf/mobility/PathCollection.hpp"
#include "dsf/base/Node.hpp"
#include "dsf/mobility/Road.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/base/AdjacencyMatrix.hpp"

#include <cassert>
#include <cstdint>
#include <filesystem>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf;
using namespace dsf::mobility;

static const auto DATA_FOLDER =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "data";

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
      graph.addNode(0, dsf::geometry::Point(0., 0.));
      graph.addNode(1, dsf::geometry::Point(-1., 0.));
      graph.addNode(2, dsf::geometry::Point(0., -1.));
      graph.addNode(3, dsf::geometry::Point(1., 1.));
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
  SUBCASE("importEdges and importNodeProperties") {
    GIVEN("A graph object") {
      RoadNetwork graph;
      WHEN("We import edges and then node properties from OSM") {
        graph.importEdges((DATA_FOLDER / "postua_edges.csv").string());
        graph.importNodeProperties((DATA_FOLDER / "postua_nodes.csv").string());
        std::ifstream fNodes{(DATA_FOLDER / "postua_nodes.csv").string()};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{(DATA_FOLDER / "postua_edges.csv").string()};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
          CHECK_EQ(graph.nCoils(), 0);
          CHECK_EQ(graph.nIntersections(), 25);
          CHECK_EQ(graph.nRoundabouts(), 1);
          CHECK_EQ(graph.nTrafficLights(), 0);
        }
        RoadNetwork graph2;
        graph2.importEdges((DATA_FOLDER / "postua_edges.geojson").string());
        THEN("Sizes are correct also with geojson") {
          CHECK_EQ(graph2.nEdges(), graph.nEdges());
          CHECK_EQ(graph2.nNodes(), graph.nNodes());
          for (auto const& [edgeId, pEdge] : graph2.edges()) {
            CHECK_EQ(*pEdge, *graph.edge(edgeId));
          }
        }
      }
      WHEN("We import many nodes and edges from OSM") {
        graph.importEdges((DATA_FOLDER / "forlì_edges.csv").string());
        graph.importNodeProperties((DATA_FOLDER / "forlì_nodes.csv").string());
        std::ifstream fNodes{(DATA_FOLDER / "forlì_nodes.csv").string()};
        // get number of lines
        std::string line;
        int nNodes{-1};  // -1 because of the header
        while (std::getline(fNodes, line)) {
          ++nNodes;
        }
        fNodes.close();
        std::ifstream fEdges{(DATA_FOLDER / "forlì_edges.csv").string()};
        int nEdges{-1};  // -1 because of the header
        while (std::getline(fEdges, line)) {
          ++nEdges;
        }
        fEdges.close();
        THEN("Sizes are correct") {
          CHECK_EQ(graph.nNodes(), nNodes);
          CHECK_EQ(graph.nEdges(), nEdges);
          CHECK_EQ(graph.nCoils(), 0);
          CHECK_EQ(graph.nIntersections(), 11100);
          CHECK_EQ(graph.nRoundabouts(), 17);
          CHECK_EQ(graph.nTrafficLights(), 30);
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
            auto& tl{graph2.node<dsf::mobility::TrafficLight>(1)};
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
  SUBCASE("add coil on a street") {
    GIVEN("A graph object with two nodes and one street") {
      RoadNetwork graph{};
      graph.addEdge(Street{0, std::make_pair(0, 1)});
      WHEN("We add a coil to the street") {
        graph.addCoil(0);
        THEN("The street has a coil") { CHECK(graph.edge(0)->hasCoil()); }
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
    graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
    // check correct import
    CHECK_EQ(graph.nNodes(), 120);
    CHECK_EQ(graph.nEdges(), 436);

    auto const& path =
        graph.allPathsTo(118, [](auto const& pEdge) { return pEdge->length(); });
    CHECK_EQ(path.size(), 119);
    CHECK_FALSE(path.contains(118));
  }
}

TEST_CASE("ShortestPath") {
  SUBCASE("Simple Path") {
    // Create a simple network: 0 -> 1 -> 2
    //                          |         ^
    //                          +-> 3 ----+
    // Path 0->1->2 has length 200, path 0->3->2 has length 300
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(1.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(2.0, 0.0));
    graph.addNode(3, dsf::geometry::Point(1.0, 1.0));

    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s12(1, std::make_pair(1, 2), 100.0);
    Street s03(2, std::make_pair(0, 3), 150.0);
    Street s32(3, std::make_pair(3, 2), 150.0);
    graph.addStreets(s01, s12, s03, s32);

    auto pathMap =
        graph.shortestPath(0, 2, [](auto const& pEdge) { return pEdge->length(); });

    // Verify the shortest path is 0 -> 1 -> 2
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 2);

    // Target node should not be in the map
    CHECK_FALSE(pathMap.contains(2));

    // Node 3 is not on the shortest path
    CHECK_FALSE(pathMap.contains(3));
  }

  SUBCASE("Same Source and Target") {
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    auto pathMap =
        graph.shortestPath(0, 0, [](auto const& pEdge) { return pEdge->length(); });

    // When source equals target, should return empty map (no hops needed)
    CHECK(pathMap.empty());
  }

  SUBCASE("No Path Exists") {
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 100.0);
    Street s23(1, std::make_pair(2, 3), 100.0);
    graph.addStreets(s01, s23);

    auto pathMap =
        graph.shortestPath(0, 3, [](auto const& pEdge) { return pEdge->length(); });

    // No path exists, should return empty map
    CHECK(pathMap.empty());
  }

  SUBCASE("Invalid Source Node") {
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    CHECK_THROWS_AS(
        graph.shortestPath(99, 1, [](auto const& pEdge) { return pEdge->length(); }),
        std::out_of_range);
  }

  SUBCASE("Invalid Target Node") {
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 100.0);
    graph.addStreet(std::move(s01));

    CHECK_THROWS_AS(
        graph.shortestPath(0, 99, [](auto const& pEdge) { return pEdge->length(); }),
        std::out_of_range);
  }

  SUBCASE("Multiple Paths - Choose Shortest") {
    // Diamond-shaped network
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 100.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap =
        graph.shortestPath(0, 3, [](auto const& pEdge) { return pEdge->length(); });

    // Verify the shortest path is 0 -> 1 -> 3 (length 20)
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 1);
    CHECK_EQ(pathMap.at(0)[0], 1);

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    // Node 2 is not on the shortest path (0->2->3 has length 110)
    CHECK_FALSE(pathMap.contains(2));
    CHECK_FALSE(pathMap.contains(3));
  }

  SUBCASE("Complex Network with Coordinates") {
    RoadNetwork graph{};
    graph.addNode(0, dsf::geometry::Point(0.0, 0.0));
    graph.addNode(1, dsf::geometry::Point(10.0, 0.0));
    graph.addNode(2, dsf::geometry::Point(0.0, 10.0));
    graph.addNode(3, dsf::geometry::Point(10.0, 10.0));

    Street s01(0, std::make_pair(0, 1), 50.0);
    Street s02(1, std::make_pair(0, 2), 50.0);
    Street s13(2, std::make_pair(1, 3), 50.0);
    Street s23(3, std::make_pair(2, 3), 50.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap =
        graph.shortestPath(0, 3, [](auto const& pEdge) { return pEdge->length(); });

    // Both paths have same length (100), so node 0 should have two next hops
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 2);
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 1) !=
          pathMap.at(0).end());
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 2) !=
          pathMap.at(0).end());

    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    REQUIRE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);
  }

  SUBCASE("Large Network") {
    RoadNetwork graph{};
    graph.importEdges((DATA_FOLDER / "manhattan_edges.csv").string());
    graph.importNodeProperties((DATA_FOLDER / "manhattan_nodes.csv").string());

    CHECK_EQ(graph.nNodes(), 120);
    CHECK_EQ(graph.nEdges(), 436);

    auto pathMap =
        graph.shortestPath(0, 119, [](auto const& pEdge) { return pEdge->length(); });

    // Verify that a path exists
    CHECK_FALSE(pathMap.empty());

    // Source should have next hops
    REQUIRE(pathMap.contains(0));
    CHECK_GT(pathMap.at(0).size(), 0);

    // Target should not be in the map
    CHECK_FALSE(pathMap.contains(119));

    // Verify connectivity: for each node in pathMap, verify edges exist to next hops
    for (auto const& [nodeId, nextHops] : pathMap) {
      for (auto const& nextHop : nextHops) {
        CHECK(graph.edge(nodeId, nextHop));
      }
    }
  }

  SUBCASE("Equivalent Paths with Threshold") {
    // Create a network with two equal-length paths
    // 0 -> 1 -> 3 (length 20)
    // 0 -> 2 -> 3 (length 20)
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s23(3, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s02, s13, s23);

    auto pathMap =
        graph.shortestPath(0, 3, [](auto const& pEdge) { return pEdge->length(); }, 0.01);

    // Check that node 0 has multiple next hops (both 1 and 2)
    REQUIRE(pathMap.contains(0));
    CHECK_EQ(pathMap.at(0).size(), 2);
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 1) !=
          pathMap.at(0).end());
    CHECK(std::find(pathMap.at(0).begin(), pathMap.at(0).end(), 2) !=
          pathMap.at(0).end());

    // Both intermediate nodes should lead to target
    REQUIRE(pathMap.contains(1));
    CHECK_EQ(pathMap.at(1).size(), 1);
    CHECK_EQ(pathMap.at(1)[0], 3);

    REQUIRE(pathMap.contains(2));
    CHECK_EQ(pathMap.at(2).size(), 1);
    CHECK_EQ(pathMap.at(2)[0], 3);

    // Test explode function to get all path combinations
    auto allPaths = pathMap.explode(0, 3);
    CHECK_EQ(allPaths.size(), 2);

    // Verify both paths are present
    bool foundPath1 = false;
    bool foundPath2 = false;
    for (auto const& path : allPaths) {
      if (path.size() == 3 && path[0] == 0 && path[1] == 1 && path[2] == 3) {
        foundPath1 = true;
      }
      if (path.size() == 3 && path[0] == 0 && path[1] == 2 && path[2] == 3) {
        foundPath2 = true;
      }
    }
    CHECK(foundPath1);
    CHECK(foundPath2);
  }

  SUBCASE("PathCollection::explode - Complex Multiple Paths") {
    /* Create a more complex network with multiple equivalent paths
          1 -> 3
        /      \
        0        5
        \      /
          2 -> 4
      All edges have equal length, so there are multiple shortest paths */
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s02(1, std::make_pair(0, 2), 10.0);
    Street s13(2, std::make_pair(1, 3), 10.0);
    Street s24(3, std::make_pair(2, 4), 10.0);
    Street s35(4, std::make_pair(3, 5), 10.0);
    Street s45(5, std::make_pair(4, 5), 10.0);
    graph.addStreets(s01, s02, s13, s24, s35, s45);

    auto pathMap =
        graph.shortestPath(0, 5, [](auto const& pEdge) { return pEdge->length(); }, 0.01);

    // Test explode function
    auto allPaths = pathMap.explode(0, 5);
    CHECK_EQ(allPaths.size(), 2);

    // Verify the two paths are: 0 -> 1 -> 3 -> 5 and 0 -> 2 -> 4 -> 5
    bool foundPath1 = false;
    bool foundPath2 = false;
    for (auto const& path : allPaths) {
      CHECK_EQ(path.size(), 4);
      CHECK_EQ(path[0], 0);
      CHECK_EQ(path[3], 5);

      if (path[1] == 1 && path[2] == 3) {
        foundPath1 = true;
      }
      if (path[1] == 2 && path[2] == 4) {
        foundPath2 = true;
      }
    }
    CHECK(foundPath1);
    CHECK(foundPath2);
  }

  SUBCASE("PathCollection::explode - Many Equivalent Paths") {
    // Create a grid-like network with many equivalent paths
    //   0 -> 1 -> 2
    //   |    |    |
    //   v    v    v
    //   3 -> 4 -> 5
    // All horizontal and vertical edges have equal length
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s03(2, std::make_pair(0, 3), 10.0);
    Street s14(3, std::make_pair(1, 4), 10.0);
    Street s25(4, std::make_pair(2, 5), 10.0);
    Street s34(5, std::make_pair(3, 4), 10.0);
    Street s45(6, std::make_pair(4, 5), 10.0);
    graph.addStreets(s01, s12, s03, s14, s25, s34, s45);

    auto pathMap =
        graph.shortestPath(0, 5, [](auto const& pEdge) { return pEdge->length(); }, 0.01);

    // Test explode function
    auto allPaths = pathMap.explode(0, 5);

    // All shortest paths from 0 to 5 have length 40 (4 edges).
    // There are 3 such shortest paths: 0->1->2->5, 0->1->4->5, and 0->3->4->5.
    CHECK_GT(allPaths.size(), 0);

    // Verify all paths start at 0 and end at 5
    for (auto const& path : allPaths) {
      CHECK_EQ(path[0], 0);
      CHECK_EQ(path[path.size() - 1], 5);
    }
  }

  SUBCASE("PathCollection::explode - Single Path") {
    // Create a simple linear network
    RoadNetwork graph{};
    Street s01(0, std::make_pair(0, 1), 10.0);
    Street s12(1, std::make_pair(1, 2), 10.0);
    Street s23(2, std::make_pair(2, 3), 10.0);
    graph.addStreets(s01, s12, s23);

    auto pathMap =
        graph.shortestPath(0, 3, [](auto const& pEdge) { return pEdge->length(); });

    // Test explode function - should return only one path
    auto allPaths = pathMap.explode(0, 3);
    CHECK_EQ(allPaths.size(), 1);

    auto const& path = allPaths.front();
    CHECK_EQ(path.size(), 4);
    CHECK_EQ(path[0], 0);
    CHECK_EQ(path[1], 1);
    CHECK_EQ(path[2], 2);
    CHECK_EQ(path[3], 3);
  }

  SUBCASE("PathCollection::explode - No Path") {
    // Create a PathCollection with no path to target
    dsf::mobility::PathCollection pathMap;
    pathMap[0] = {1, 2};
    pathMap[1] = {3};
    pathMap[2] = {4};
    // No path from 0 to 5

    auto allPaths = pathMap.explode(0, 5);
    CHECK_EQ(allPaths.size(), 0);
  }

  SUBCASE("PathCollection::explode - Same Source and Target") {
    // Test when source equals target
    dsf::mobility::PathCollection pathMap;
    pathMap[0] = {1, 2};
    pathMap[1] = {3};

    auto allPaths = pathMap.explode(0, 0);
    CHECK_EQ(allPaths.size(), 1);

    auto const& path = allPaths.front();
    CHECK_EQ(path.size(), 1);
    CHECK_EQ(path[0], 0);
  }

  SUBCASE("Import GeoJSON with Transition Probabilities") {
    GIVEN("A GeoJSON file with transition probabilities") {
      RoadNetwork graph;

      WHEN("We import the GeoJSON file") {
        graph.importEdges((DATA_FOLDER / "test_transition_probs.geojson").string());

        THEN("The graph is constructed correctly") {
          CHECK_EQ(graph.nEdges(), 4);
          CHECK_EQ(graph.nNodes(), 5);
        }

        THEN("Transition probabilities are correctly imported for street 1") {
          auto const& street1 = graph.edge(1);
          auto const& transProbs1 = street1->transitionProbabilities();

          CHECK_EQ(transProbs1.size(), 2);
          CHECK(transProbs1.contains(2));
          CHECK(transProbs1.contains(3));
          CHECK_EQ(transProbs1.at(2), doctest::Approx(0.6));
          CHECK_EQ(transProbs1.at(3), doctest::Approx(0.4));
        }

        THEN("Transition probabilities are correctly imported for street 3") {
          auto const& street3 = graph.edge(3);
          auto const& transProbs3 = street3->transitionProbabilities();

          CHECK_EQ(transProbs3.size(), 1);
          CHECK(transProbs3.contains(4));
          CHECK_EQ(transProbs3.at(4), doctest::Approx(1.0));
        }

        THEN("Streets without transition probabilities have empty maps") {
          auto const& street2 = graph.edge(2);
          auto const& transProbs2 = street2->transitionProbabilities();
          CHECK_EQ(transProbs2.size(), 0);

          auto const& street4 = graph.edge(4);
          auto const& transProbs4 = street4->transitionProbabilities();
          CHECK_EQ(transProbs4.size(), 0);
        }

        THEN("Other street properties are imported correctly") {
          auto const& street1 = graph.edge(1);
          CHECK_EQ(street1->id(), 1);
          CHECK_EQ(street1->source(), 0);
          CHECK_EQ(street1->target(), 1);
          CHECK_EQ(street1->length(), doctest::Approx(100.5));
          CHECK_EQ(street1->maxSpeed(), doctest::Approx(50.0 / 3.6));
          CHECK_EQ(street1->nLanes(), 2);
          CHECK_EQ(street1->name(), "Test Street A");
        }
      }
    }
  }
}
