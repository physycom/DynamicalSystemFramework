#include <cstdint>

#include "FirstOrderDynamics.hpp"
#include "RoadNetwork.hpp"
#include "Itinerary.hpp"
#include "Street.hpp"

#include "doctest.h"

using namespace dsm;

TEST_CASE("Measurement") {
  SUBCASE("STL vector") {
    std::vector<float> data(100);
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
  }
  SUBCASE("STL array") {
    std::array<float, 100> data;
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
  }
  SUBCASE("STL span") {
    auto p = std::make_unique_for_overwrite<float[]>(100);
    std::span<float> data(p.get(), 100);
    std::iota(data.begin(), data.end(), 0.f);

    Measurement<float> m(data);
    CHECK_EQ(m.mean, 49.5f);
    CHECK_EQ(m.std, doctest::Approx(28.8661f));
  }
}

TEST_CASE("FirstOrderDynamics") {
  // dsm::Logger::setLogLevel(dsm::log_level_t::DEBUG);
  SUBCASE("Constructor") {
    GIVEN("A graph object") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dsm");
      graph.buildAdj();
      WHEN("A dynamics object is created") {
        FirstOrderDynamics dynamics{
            graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
        THEN("The node and the street sets are the same") {
          CHECK_EQ(dynamics.graph().nNodes(), 3);
          CHECK_EQ(dynamics.graph().nEdges(), 4);
        }
        THEN("The mean speed, density, flow and travel time are 0") {
          // CHECK_EQ(dynamics.agentMeanSpeed().mean, 0.);
          // CHECK_EQ(dynamics.agentMeanSpeed().std, 0.);
          CHECK_EQ(dynamics.streetMeanDensity().mean, 0.);
          CHECK_EQ(dynamics.streetMeanDensity().std, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().mean, 0.);
          CHECK_EQ(dynamics.streetMeanFlow().std, 0.);
          CHECK_EQ(dynamics.meanTravelTime().mean, 0.);
          CHECK_EQ(dynamics.meanTravelTime().std, 0.);
        }
      }
      WHEN("We transform a node into a traffic light and create the dynamics") {
        auto& tl = graph.makeTrafficLight(0, 2);
        FirstOrderDynamics dynamics{graph, false, 69};
        THEN("The node is a traffic light") {
          CHECK(dynamics.graph().node(0)->isTrafficLight());
          CHECK_EQ(tl.cycleTime(), 2);
        }
      }
      WHEN("We transform a node into a roundabout and create the dynamics") {
        graph.makeRoundabout(0);
        FirstOrderDynamics dynamics{graph, false, 69};
        THEN("The node is a roundabout") {
          CHECK(dynamics.graph().node(0)->isRoundabout());
        }
      }
      WHEN("We transorm a street into a spire and create the dynamcis") {
        graph.makeSpireStreet(8);
        FirstOrderDynamics dynamics{graph, false, 69};
        THEN("The street is a spire") { CHECK(dynamics.graph().edge(8)->isSpire()); }
      }
    }
  }
  SUBCASE("setDestinationNodes") {
    GIVEN("A dynamics object and a destination node") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dat");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      WHEN("We add a span of destination nodes") {
        std::array<uint32_t, 3> nodes{0, 1, 2};
        dynamics.setDestinationNodes(nodes);
        THEN("The destination nodes are added") {
          const auto& itineraries = dynamics.itineraries();
          CHECK_EQ(itineraries.size(), nodes.size());
          for (uint16_t i{0}; i < nodes.size(); ++i) {
            CHECK_EQ(itineraries.at(i)->destination(), nodes.at(i));
          }
        }
      }
    }
  }
  SUBCASE("addAgent") {
    GIVEN("A dynamics object, a source node and a destination node") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dsm");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      WHEN("We add the agent") {
        dynamics.addAgent(2, 0);
        THEN("The agent is added") {
          CHECK_EQ(dynamics.nAgents(), 1);
          const auto& agent = dynamics.agents().at(0);
          // CHECK_EQ(agent->id(), 0);
          CHECK_EQ(agent->srcNodeId().value(), 0);
          CHECK_EQ(agent->itineraryId(), 2);
        }
      }
    }
  }
  SUBCASE("addAgentsUniformly") {
    GIVEN("A dynamics object and an itinerary") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dsm");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      WHEN("We add a random agent") {
        dynamics.addItinerary(2, 2);
        dynamics.addAgentsUniformly(1);
        THEN(
            "The number of agents is 1 and the destination is the same as the "
            "itinerary") {
          CHECK_EQ(dynamics.nAgents(), 1);
        }
      }
    }
    GIVEN("A dynamics object and many itineraries") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dsm");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      dynamics.addItinerary(1, 1);
      WHEN("We add many agents") {
        dynamics.addAgentsUniformly(3);
        THEN(
            "The number of agents is 3, the destination and the street is the "
            "same as "
            "the itinerary") {
          CHECK_EQ(dynamics.nAgents(), 3);
#ifdef __APPLE__
          CHECK_EQ(dynamics.graph().edge(1)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(1)->movingAgents().top()->itineraryId(), 2);
          CHECK_EQ(dynamics.graph().edge(6)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(6)->movingAgents().top()->itineraryId(), 1);
          CHECK_EQ(dynamics.graph().edge(8)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(8)->movingAgents().top()->itineraryId(), 2);
#else
          CHECK_EQ(dynamics.graph().edge(1)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(1)->movingAgents().top()->itineraryId(), 1);
          CHECK_EQ(dynamics.graph().edge(3)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(3)->movingAgents().top()->itineraryId(), 2);
          CHECK_EQ(dynamics.graph().edge(8)->nAgents(), 1);
          CHECK_EQ(dynamics.graph().edge(8)->movingAgents().top()->itineraryId(), 2);
#endif
        }
      }
    }
  }
  SUBCASE("addAgentsRandomly") {
    GIVEN("A graph object") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dat");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      WHEN("We add one agent for existing itinerary") {
        std::unordered_map<uint32_t, double> src{{0, 1.}};
        std::unordered_map<uint32_t, double> dst{{2, 1.}};
        dynamics.addItinerary(2, 2);
        dynamics.addAgentsRandomly(1, src, dst);
        THEN("The agents are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.itineraries()
                       .at(dynamics.agents().at(0)->itineraryId())
                       ->destination(),
                   2);
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 0);
        }
      }
      WHEN("We add agents for existing itineraries") {
        std::unordered_map<uint32_t, double> src{{1, 0.3}, {27, 0.3}, {118, 0.4}};
        std::unordered_map<uint32_t, double> dst{{14, 0.3}, {102, 0.3}, {107, 0.4}};
        std::vector<dsm::Id> destinations{14, 102, 107};
        dynamics.setDestinationNodes(destinations);
        dynamics.addAgentsRandomly(3, src, dst);
        THEN("The agents are correctly set") {
          CHECK_EQ(dynamics.nAgents(), 3);
          CHECK_EQ(dynamics.itineraries()
                       .at(dynamics.agents().at(0)->itineraryId())
                       ->destination(),
                   107);
          CHECK_EQ(dynamics.agents().at(0)->srcNodeId().value(), 27);
          CHECK_EQ(dynamics.itineraries()
                       .at(dynamics.agents().at(1)->itineraryId())
                       ->destination(),
                   14);
          CHECK_EQ(dynamics.agents().at(1)->srcNodeId().value(), 1);
          CHECK_EQ(dynamics.itineraries()
                       .at(dynamics.agents().at(2)->itineraryId())
                       ->destination(),
                   14);
          CHECK_EQ(dynamics.agents().at(2)->srcNodeId().value(), 118);
        }
      }
    }
  }
  SUBCASE("addRandomAgents") {
    GIVEN("A dynamics object") {
      auto const p{0.1};
      auto const n{100};
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dat", false);
      graph.buildAdj();
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.setPassageProbability(p);
      WHEN("We add some agent") {
        dynamics.addAgents(n);
        THEN("The number of agents is correct") { CHECK_EQ(dynamics.nAgents(), 100); }
        THEN("If we evolve the dynamics agent disappear gradually") {
          // for (auto i{0}; i < 40; ++i) {
          //   dynamics.evolve(false);
          // }
          // CHECK(dynamics.nAgents() < n);
        }
      }
    }
  }
  SUBCASE("addAgents") {
    GIVEN("A dynamics object and one itinerary") {
      auto graph = RoadNetwork{};
      graph.importMatrix("./data/matrix.dsm");
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      WHEN("We add an agent with itinerary 2") {
        dynamics.addAgent(2, 0);
        THEN(
            "The number of agents is 1 and the destination is the same as the "
            "itinerary") {
          CHECK_EQ(dynamics.nAgents(), 1);
          CHECK_EQ(dynamics.itineraries()
                       .at(dynamics.agents().at(0)->itineraryId())
                       ->destination(),
                   2);
        }
      }
      WHEN("We add 69 agents with itinerary 0") {
        dynamics.addAgents(69, 0);
        THEN("The number of agents is 69") { CHECK_EQ(dynamics.nAgents(), 69); }
      }
    }
  }
  //   SUBCASE("Add too many agents") {
  //     GIVEN("A simple graph with two nodes and only one street") {
  //       Street s{0, std::make_pair(0, 1), 2.};  // Capacity of 1 agent
  //       RoadNetwork graph2;
  //       graph2.addStreets(s);
  //       graph2.buildAdj();
  //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
  //       dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(0, 1)));
  //       dynamics.updatePaths();
  //       dynamics.addAgentsUniformly(1);
  //       WHEN("We add more than one agent") {
  //         THEN("It throws") {
  //           CHECK_THROWS_AS(dynamics.addAgentsUniformly(1), std::overflow_error);
  //           CHECK_THROWS_AS(dynamics.addAgent(1, 0, 0), std::overflow_error);
  //           CHECK_THROWS_AS(dynamics.addAgent(std::make_unique<Agent>(Agent(1, 0))),
  //                           std::overflow_error);
  //         }
  //       }
  //     }
  //   }
  SUBCASE("Update paths") {
    GIVEN("A dynamics object, many streets and an itinerary") {
      Street s1{0, std::make_pair(0, 1), 2.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 2), 10.};
      RoadNetwork graph2;
      graph2.addStreets(s1, s2, s3);
      graph2.buildAdj();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0.0, dsm::weight_functions::streetLength, 1.};
      WHEN("We add an itinerary and update the paths") {
        dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(0, 2)));
        dynamics.updatePaths();
        THEN(
            "The number of itineraries is 1 and the path is updated and "
            "correctly "
            "formed") {
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK(dynamics.itineraries().at(0)->path()->operator()(0, 1));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(1, 2));
          CHECK_FALSE(dynamics.itineraries().at(0)->path()->operator()(0, 2));
          for (auto const& it : dynamics.itineraries()) {
            auto const& path = it.second->path();
            for (uint16_t i{0}; i < path->n(); ++i) {
              if (i == it.second->destination()) {
                CHECK_FALSE(path->getRow(i).size());
              } else {
                CHECK(path->getRow(i).size());
              }
            }
          }
        }
      }
    }
    GIVEN(
        "A dynamics objects, many streets and many itinearies with same "
        "destination") {
      RoadNetwork graph2{};
      graph2.importMatrix("./data/matrix.dat");
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(0, 118);
      dynamics.addItinerary(1, 118);
      dynamics.addItinerary(2, 118);
      dynamics.addItinerary(3, 118);
      dynamics.updatePaths();
      for (auto const& it : dynamics.itineraries()) {
        auto const& path = it.second->path();
        for (uint16_t i{0}; i < path->n(); ++i) {
          if (i == it.second->destination()) {
            CHECK_FALSE(path->getRow(i).size());
          } else {
            CHECK(path->getRow(i).size());
          }
        }
      }
    }
    GIVEN("A dynamics objects, many streets and an itinerary with bifurcations") {
      Street s1{0, std::make_pair(0, 1), 5.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 3), 5.};
      Street s4{3, std::make_pair(3, 2), 5.};
      RoadNetwork graph;
      graph.addStreets(s1, s2, s3, s4);
      graph.buildAdj();
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(0, 2);
      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The path is updated and correctly formed") {
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK_EQ(dynamics.itineraries().at(0)->path()->size(), 4);
          CHECK_EQ(dynamics.itineraries().at(0)->path()->n(), 4);
          CHECK(dynamics.itineraries().at(0)->path()->operator()(0, 1));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(1, 2));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(0, 3));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(3, 2));
          for (auto const& it : dynamics.itineraries()) {
            auto const& path = it.second->path();
            for (uint16_t i{0}; i < path->n(); ++i) {
              if (i == it.second->destination()) {
                CHECK_FALSE(path->getRow(i).size());
              } else {
                CHECK(path->getRow(i).size());
              }
            }
          }
        }
      }
    }
    GIVEN(
        "A dynamics objects, many streets and an itinerary with bifurcations (TIME "
        "WEIGHTED)") {
      Street s1{0, std::make_pair(0, 1), 5., 50.};
      Street s2{1, std::make_pair(1, 2), 7., 70.};
      Street s3{2, std::make_pair(0, 3), 9., 90.};
      Street s4{3, std::make_pair(3, 2), 10., 100.};
      RoadNetwork graph;
      graph.addStreets(s1, s2, s3, s4);
      graph.buildAdj();
      FirstOrderDynamics dynamics{
          graph, false, 69, 0., dsm::weight_functions::streetTime, 1.};
      // dynamics.setWeightFunction(dsm::weight_functions::streetTime);
      dynamics.addItinerary(0, 2);
      WHEN("We update the paths") {
        dynamics.updatePaths();
        THEN("The path is updated and correctly formed") {
          CHECK_EQ(dynamics.itineraries().size(), 1);
          CHECK_EQ(dynamics.itineraries().at(0)->path()->size(), 4);
          CHECK_EQ(dynamics.itineraries().at(0)->path()->n(), 4);
          CHECK(dynamics.itineraries().at(0)->path()->operator()(0, 1));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(1, 2));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(0, 3));
          CHECK(dynamics.itineraries().at(0)->path()->operator()(3, 2));
          for (auto const& it : dynamics.itineraries()) {
            auto const& path = it.second->path();
            for (uint16_t i{0}; i < path->n(); ++i) {
              if (i == it.second->destination()) {
                CHECK_FALSE(path->getRow(i).size());
              } else {
                CHECK(path->getRow(i).size());
              }
            }
          }
        }
      }
    }
  }
  SUBCASE("Evolve") {
    GIVEN("A dynamics object and an itinerary") {
      Street s1{0, std::make_pair(0, 1), 2.};
      Street s2{1, std::make_pair(1, 2), 5.};
      Street s3{2, std::make_pair(0, 2), 10.};
      RoadNetwork graph;
      graph.addStreets(s1, s2, s3);
      graph.buildAdj();
      FirstOrderDynamics dynamics{
          graph, false, 69, 0.0, dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      WHEN("We add an agent randomly and evolve the dynamics") {
        dynamics.addAgent(2, 0);
        auto const& streets{dynamics.graph().edges()};
        dynamics.evolve(false);  // Agent goes into node 0
        dynamics.evolve(false);  // Agent goes from node 0 to street 0->1
        THEN("The agent evolves") {
          CHECK_EQ(streets.at(1)->movingAgents().size(), 1);
          auto const& pAgent{streets.at(1)->movingAgents().top()};
          CHECK(pAgent);
          CHECK_EQ(dynamics.time() - pAgent->spawnTime(), dynamics.time());
          CHECK_EQ(pAgent->freeTime(), dynamics.time());
          CHECK_EQ(pAgent->streetId().value(), 1);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve(false);  // Agent enqueues on street 0->1
        THEN("The agent evolves again, changing street") {
          auto const& pAgent{streets.at(1)->queue(0).front()};
          CHECK_EQ(dynamics.time() - pAgent->spawnTime(), dynamics.time());
          CHECK_EQ(pAgent->freeTime() + 1, dynamics.time());
          CHECK_EQ(pAgent->streetId().value(), 1);
          CHECK_EQ(pAgent->speed(), 0.);
        }
        dynamics.evolve(false);  // Agent changes street
        THEN("The agent evolves again, changing street") {
          auto const& pAgent{streets.at(5)->movingAgents().top()};
          CHECK_EQ(dynamics.time() - pAgent->spawnTime(), dynamics.time());
          CHECK_EQ(pAgent->freeTime(), dynamics.time());
          CHECK_EQ(pAgent->streetId().value(), 5);
          CHECK_EQ(pAgent->speed(), 13.8888888889);
        }
        dynamics.evolve(false);  // Enqueues on street 5
        dynamics.evolve(false);  // Goes into destination nodes and gets killed
        THEN("And again, reaching the destination") { CHECK_EQ(dynamics.nAgents(), 0); }
      }
    }
    //     GIVEN("A dynamics object, an itinerary and an agent") {
    //       Street s1{0, std::make_pair(0, 1), 13.8888888889};
    //       Street s2{1, std::make_pair(1, 0), 13.8888888889};
    //       RoadNetwork graph2;
    //       graph2.addStreets(s1, s2);
    //       graph2.buildAdj();
    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(0, 1)));
    //       dynamics.updatePaths();
    //       dynamics.addAgent(0, 0, 0);
    //       WHEN("We evolve the dynamics") {
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         THEN("The agent evolves") {
    //           CHECK_EQ(dynamics.time() - dynamics.agents().at(0)->spawnTime(), 2);
    //           CHECK_EQ(dynamics.agents().at(0)->freeTime(), dynamics.time());
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 1);
    //           CHECK_EQ(dynamics.agents().at(0)->speed(), 13.8888888889);
    //           CHECK_EQ(dynamics.agents().at(0)->distance(), 13.8888888889);
    //         }
    //         dynamics.evolve(false);
    //         THEN("The agent reaches the destination") { CHECK(dynamics.agents().empty()); }
    //       }
    //     }
    //     GIVEN("A dynamics object, an itinerary and an agent") {
    //       Street s1{0, std::make_pair(0, 1), 13.8888888889};
    //       Street s2{1, std::make_pair(1, 0), 13.8888888889};
    //       RoadNetwork graph2;
    //       graph2.addStreets(s1, s2);
    //       graph2.buildAdj();
    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.addItinerary(1, 1);
    //       dynamics.updatePaths();
    //       dynamics.addAgent(0, 1, 0);
    //       WHEN("We evolve the dynamics with reinsertion") {
    //         dynamics.evolve(true);
    //         dynamics.evolve(true);
    //         THEN("The agent has correct values") {
    //           CHECK_EQ(dynamics.time() - dynamics.agents().at(0)->spawnTime(), 2);
    //           CHECK_EQ(dynamics.agents().at(0)->freeTime(), dynamics.time());
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 1);
    //           CHECK_EQ(dynamics.agents().at(0)->speed(), 13.8888888889);
    //           CHECK_EQ(dynamics.agents().at(0)->distance(), 13.8888888889);
    //         }
    //         dynamics.evolve(true);
    //         THEN("The agent is reinserted") {
    //           CHECK_EQ(dynamics.nAgents(), 1);
    //           CHECK_EQ(dynamics.time() - dynamics.agents().at(0)->spawnTime(), 1);
    //           CHECK(dynamics.agents().at(0)->freeTime() < dynamics.time());
    //           CHECK_FALSE(dynamics.agents().at(0)->streetId().has_value());
    //           CHECK_EQ(dynamics.agents().at(0)->speed(), 0.);
    //         }
    //       }
    //     }
    //     GIVEN("A simple network and an agent with forced itinerary") {
    //       Street s0_1{1, std::make_pair(0, 1), 30., 15.};
    //       Street s1_0{3, std::make_pair(1, 0), 30., 15.};
    //       Street s1_2{5, std::make_pair(1, 2), 30., 15.};
    //       Street s2_1{7, std::make_pair(2, 1), 30., 15.};
    //       RoadNetwork graph2;
    //       graph2.addStreets(s0_1, s1_0, s1_2, s2_1);
    //       graph2.buildAdj();
    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.setDestinationNodes({1, 2});
    //       std::vector<dsm::Id> trip{2, 1};
    //       dynamics.addAgent(0, trip, 0);
    //       auto const& pAgent{dynamics.agents().at(0)};
    //       WHEN("We evolve the dynamics") {
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         THEN("The agent goes first into node 2") {
    //           CHECK_EQ(pAgent->streetId().value(), 5);
    //           CHECK_EQ(pAgent->distance(), 60.);
    //         }
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         THEN("The agent goes then to node 1") {
    //           CHECK_EQ(pAgent->streetId().value(), 7);
    //           CHECK_EQ(pAgent->distance(), 90.);
    //         }
    //         dynamics.evolve(false);
    //         THEN("The agent reaches the destination") { CHECK(dynamics.agents().empty()); }
    //       }
    //     }
  }
  SUBCASE("TrafficLights") {
    GIVEN(
        "A dynamics object, a network with traffic lights, an itinerary and "
        "an agent") {
      Street s1{1, std::make_pair(0, 1), 30., 15.};
      Street s2{7, std::make_pair(1, 2), 30., 15.};
      Street s3{16, std::make_pair(3, 1), 30., 15.};
      Street s4{9, std::make_pair(1, 4), 30., 15.};
      RoadNetwork graph2;
      graph2.addNode<TrafficLight>(1, 4);
      graph2.addStreets(s1, s2, s3, s4);
      graph2.buildAdj();
      auto& tl = graph2.node<TrafficLight>(1);
      tl.setCycle(1, dsm::Direction::RIGHT, {2, 0});
      tl.setCycle(7, dsm::Direction::RIGHT, {2, 0});
      tl.setCycle(16, dsm::Direction::RIGHT, {2, 2});
      tl.setCycle(9, dsm::Direction::RIGHT, {2, 2});
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        auto const& streets{dynamics.graph().edges()};
        THEN(
            "The agent is ready to go through the traffic light at time 3 but "
            "the "
            "traffic light is red"
            " until time 4, so the agent waits until time 4") {
          for (uint8_t i{0}; i < 5; ++i) {
            dynamics.evolve(false);
            if (i < 3) {
              CHECK_EQ(streets.at(1)->nAgents(), 1);
            } else {
              CHECK_EQ(streets.at(7)->nAgents(), 1);
            }
            if (i == 2) {
              CHECK_EQ(streets.at(1)->queue(0).front()->distance(), 30.);
            }
          }
          CHECK_EQ(streets.at(7)->queue(0).front()->distance(), 60.);
        }
      }
    }
    //     GIVEN(
    //         "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
    //         "streets") {
    //       // Streets
    //       Street s0_1{1, std::make_pair(0, 1), 30., 15., 3};
    //       Street s1_0{5, std::make_pair(1, 0), 30., 15., 3};
    //       Street s1_2{7, std::make_pair(1, 2), 30., 15., 3};
    //       Street s2_1{11, std::make_pair(2, 1), 30., 15., 3};

    //       Street s3_1{8, std::make_pair(3, 1), 30., 15.};
    //       Street s1_3{16, std::make_pair(1, 3), 30., 15.};
    //       Street s4_1{21, std::make_pair(4, 1), 30., 15.};
    //       Street s1_4{9, std::make_pair(1, 4), 30., 15.};

    //       RoadNetwork graph2;
    //       {
    //         graph2.addNode<TrafficLight>(1, 6, std::make_pair(0, 0));
    //         auto& tl = graph2.node<TrafficLight>(1);
    //         tl.setCycle(1, dsm::Direction::RIGHTANDSTRAIGHT, {2, 2});
    //         tl.setCycle(1, dsm::Direction::LEFT, {1, 4});
    //         tl.setCycle(11, dsm::Direction::ANY, {3, 2});
    //         tl.setComplementaryCycle(8, 11);
    //         tl.setComplementaryCycle(21, 11);
    //       }
    //       graph2.addStreets(s0_1, s1_0, s1_2, s2_1, s3_1, s1_3, s4_1, s1_4);
    //       auto const& nodes = graph2.nodes();
    //       nodes.at(0)->setCoords({0., -1.});
    //       nodes.at(2)->setCoords({0., 1.});
    //       nodes.at(3)->setCoords({-1., 0.});
    //       nodes.at(4)->setCoords({1., 0.});
    //       graph2.buildAdj();
    //       graph2.adjustNodeCapacities();

    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.setDestinationNodes({0, 2, 3, 4});

    //       WHEN("We add agents and make the system evolve") {
    //         dynamics.addAgent(0, 2, 0);
    //         dynamics.addAgent(1, 4, 0);
    //         dynamics.evolve(false);  // Counter 0
    //         THEN("The agents are not yet on the streets") {
    //           CHECK_FALSE(dynamics.agents().at(0)->streetId().has_value());
    //           CHECK_FALSE(dynamics.agents().at(1)->streetId().has_value());
    //         }
    //         dynamics.evolve(false);  // Counter 1
    //         THEN("The agents are correctly placed") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 1);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 1);
    //         }
    //         dynamics.evolve(false);  // Counter 2
    //         dynamics.evolve(false);  // Counter 3
    //         THEN("The agent 0 passes and agent 1 waits") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 7);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 1);
    //         }
    //         dynamics.evolve(false);  // Counter 4
    //         THEN("The agent 1 passes") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 7);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 9);
    //         }
    //       }
    //     }
    //     GIVEN(
    //         "A traffic light managing an intersection with 4 3-lanes streets and 4 1-lane "
    //         "streets") {
    //       // Streets
    //       Street s0_1{1, std::make_pair(0, 1), 30., 15., 3};
    //       Street s1_0{5, std::make_pair(1, 0), 30., 15., 3};
    //       Street s1_2{7, std::make_pair(1, 2), 30., 15., 3};
    //       Street s2_1{11, std::make_pair(2, 1), 30., 15., 3};

    //       Street s3_1{8, std::make_pair(3, 1), 30., 15.};
    //       Street s1_3{16, std::make_pair(1, 3), 30., 15.};
    //       Street s4_1{21, std::make_pair(4, 1), 30., 15.};
    //       Street s1_4{9, std::make_pair(1, 4), 30., 15.};

    //       RoadNetwork graph2;
    //       {
    //         graph2.addNode<TrafficLight>(1, 6, std::make_pair(0, 0));
    //         auto& tl = graph2.node<TrafficLight>(1);
    //         // Now testing red light = NO PHASE
    //         tl.setCycle(1, dsm::Direction::RIGHTANDSTRAIGHT, {2, 0});
    //         tl.setCycle(1, dsm::Direction::LEFT, {1, 2});
    //         tl.setCycle(11, dsm::Direction::ANY, {3, 0});
    //         tl.setComplementaryCycle(8, 11);
    //         tl.setComplementaryCycle(21, 11);
    //       }
    //       graph2.addStreets(s0_1, s1_0, s1_2, s2_1, s3_1, s1_3, s4_1, s1_4);
    //       auto const& nodes = graph2.nodes();
    //       nodes.at(0)->setCoords({0., -1.});
    //       nodes.at(2)->setCoords({0., 1.});
    //       nodes.at(3)->setCoords({-1., 0.});
    //       nodes.at(4)->setCoords({1., 0.});
    //       graph2.buildAdj();
    //       graph2.adjustNodeCapacities();

    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.setDestinationNodes({0, 2, 3, 4});

    //       WHEN("We add agents and make the system evolve") {
    //         dynamics.addAgent(0, 2, 0);
    //         dynamics.addAgent(1, 4, 0);
    //         dynamics.evolve(false);  // Counter 0
    //         dynamics.evolve(false);  // Counter 1
    //         THEN("The agents are correctly placed") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 1);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 1);
    //         }
    //         dynamics.evolve(false);  // Counter 2
    //         dynamics.evolve(false);  // Counter 3
    //         THEN("The agents are still") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 1);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 1);
    //         }
    //         dynamics.evolve(false);  // Counter 4
    //         dynamics.evolve(false);  // Counter 5
    //         dynamics.evolve(false);  // Counter 0
    //         THEN("The agent 0 passes and agent 1 waits") {
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 7);
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 1);
    //         }
    //         dynamics.evolve(false);  // Counter 1
    //         dynamics.evolve(false);  // Counter 2
    //         THEN("The agent 1 passes") {
    //           CHECK_EQ(dynamics.agents().at(1)->streetId().value(), 9);
    //         }
    //       }
    //     }
    //     SUBCASE("Traffic Lights optimization algorithm") {
    //       GIVEN("A dynamics object with a traffic light intersection") {
    //         double length{90.}, max_speed{15.};
    //         Street s_01{1, std::make_pair(0, 1), length, max_speed};
    //         Street s_10{5, std::make_pair(1, 0), length, max_speed};
    //         Street s_12{7, std::make_pair(1, 2), length, max_speed};
    //         Street s_21{11, std::make_pair(2, 1), length, max_speed};
    //         Street s_13{8, std::make_pair(1, 3), length, max_speed};
    //         Street s_31{16, std::make_pair(3, 1), length, max_speed};
    //         Street s_14{9, std::make_pair(1, 4), length, max_speed};
    //         Street s_41{21, std::make_pair(4, 1), length, max_speed};
    //         RoadNetwork graph2;
    //         graph2.addStreets(s_01, s_10, s_12, s_21, s_13, s_31, s_14, s_41);
    //         graph2.buildAdj();
    //         auto& tl = graph2.makeTrafficLight(1, 8, 3);
    //         tl.addStreetPriority(1);
    //         tl.addStreetPriority(11);
    //         tl.setCycle(1, dsm::Direction::ANY, {4, 0});
    //         tl.setCycle(11, dsm::Direction::ANY, {4, 0});
    //         tl.setComplementaryCycle(16, 11);
    //         tl.setComplementaryCycle(21, 11);
    //         FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //         dynamics.setDestinationNodes({0, 2, 3, 4});
    //         WHEN("We evolve the dynamics and optimize traffic lights") {
    //           dynamics.addAgents(7, 0, 2);
    //           dynamics.addAgents(7, 2, 0);
    //           dynamics.setDataUpdatePeriod(4);
    //           for (int i = 0; i < 9; ++i) {
    //             dynamics.evolve(false);
    //           }
    //           dynamics.optimizeTrafficLights(0, dsm::TrafficLightOptimization::SINGLE_TAIL);
    //           THEN("Green and red time are different") {
    //             CHECK(tl.meanGreenTime(true) > tl.meanGreenTime(false));
    //           }
    //         }
    //         WHEN(
    //             "We evolve the dynamics and optimize traffic lights with outgoing "
    //             "streets "
    //             "full") {
    //           dynamics.addAgents(5, 0, 1);
    //           dynamics.addAgents(5, 2, 1);
    //           dynamics.addAgents(5, 3, 1);
    //           dynamics.addAgents(5, 4, 1);
    //           dynamics.setDataUpdatePeriod(8);
    //           for (int i = 0; i < 15; ++i) {
    //             dynamics.evolve(false);
    //           }
    //           dynamics.optimizeTrafficLights(0, dsm::TrafficLightOptimization::SINGLE_TAIL);
    //           THEN("Green and red time are equal") {
    //             CHECK_EQ(tl.meanGreenTime(true), tl.meanGreenTime(false));
    //           }
    //         }
    //       }
    //     }
  }
  SUBCASE("Roundabout") {
    GIVEN(
        "A dynamics object with four streets, one agent for each street, two "
        "itineraries "
        "and a roundabout") {
      Road::setMeanVehicleLength(10.);
      Street s1{1, std::make_pair(0, 1), 10., 10.};
      Street s2{7, std::make_pair(2, 1), 10., 10.};
      Street s3{3, std::make_pair(1, 0), 10., 10.};
      Street s4{5, std::make_pair(1, 2), 10., 10.};
      RoadNetwork graph2;
      graph2.addStreets(s1, s2, s3, s4);
      graph2.buildAdj();
      auto& rb = graph2.makeRoundabout(1);
      graph2.adjustNodeCapacities();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(0, 0);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(2, 0);
      dynamics.addAgent(0, 2);
      WHEN(
          "We evolve the dynamics adding an agent on the path of the agent "
          "with "
          "priority") {
        dynamics.evolve(false);  // Agents into sources
        dynamics.evolve(false);  // Agents from sources to streets
        dynamics.addAgent(2, 1);
        dynamics.evolve(false);  // Agents into queues, other agent into roundabout
        dynamics.evolve(false);  // Agents from queues to roundabout
        auto const& streets{dynamics.graph().edges()};
        THEN("The agents are trapped into the roundabout") {
          CHECK_EQ(streets.at(1)->nAgents(), 0);
          CHECK_EQ(streets.at(5)->nAgents(), 1);
          CHECK_EQ(streets.at(7)->nAgents(), 1);
          CHECK_EQ(rb.agents().size(), 1);
        }
        dynamics.evolve(false);
        THEN("The agent with priority leaves the roundabout") {
          CHECK_EQ(streets.at(3)->nAgents(), 0);
          CHECK_EQ(streets.at(5)->nAgents(), 1);
          CHECK_EQ(streets.at(7)->nAgents(), 0);
          CHECK_EQ(rb.agents().size(), 2);
        }
        dynamics.evolve(false);
        THEN("The agent with priority leaves the roundabout") {
          CHECK_EQ(streets.at(3)->nAgents(), 1);
          CHECK_EQ(streets.at(5)->nAgents(), 1);
          CHECK(rb.agents().empty());
        }
      }
    }
  }
  SUBCASE("Travelled distance") {
    GIVEN("A dynamics with a two-streets network and an agent") {
      Street s1{0, std::make_pair(0, 1), 3.};
      Street s2{1, std::make_pair(1, 2), 1.};
      RoadNetwork graph2;
      graph2.addStreets(s1, s2);
      graph2.buildAdj();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent has travelled the correct distance") {
          auto const& pAgent{dynamics.graph().edge(1)->queue(0).front()};
          CHECK_EQ(dynamics.time() - pAgent->spawnTime(), 3);
          CHECK_EQ(pAgent->freeTime() + 1, dynamics.time());
          CHECK_EQ(pAgent->streetId().value(), 1);
          CHECK_EQ(pAgent->speed(), 0.);
          CHECK_EQ(pAgent->distance(), 3.);
        }
      }
    }
    //     GIVEN(
    //         "A dynamics with one stochastic street and one normal street network and an "
    //         "agent") {
    //       Street s1{0, std::make_pair(0, 1), 3.};
    //       Street s2{1, std::make_pair(1, 2), 1.};
    //       RoadNetwork graph2;
    //       graph2.addStreets(s1, s2);
    //       graph2.buildAdj();
    //       graph2.makeStochasticStreet(1, 0.3);
    //       FirstOrderDynamics dynamics{graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
    //       dynamics.addItinerary(2, 2);
    //       dynamics.updatePaths();
    //       dynamics.addAgent(0, 2, 0);
    //       WHEN("We evolve the dynamics") {
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    // #ifndef __APPLE__
    //         THEN("The agent has travelled the correct distance") {
    //           CHECK_EQ(dynamics.time() - dynamics.agents().at(0)->spawnTime(), 6);
    //           CHECK_EQ(dynamics.agents().at(0)->freeTime(), dynamics.time());
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 5);
    //           CHECK_EQ(dynamics.agents().at(0)->speed(), 13.8888888889);
    //           CHECK_EQ(dynamics.agents().at(0)->distance(), 4.);
    //         }
    // #else
    //         dynamics.evolve(false);
    //         dynamics.evolve(false);
    //         THEN("The agent has travelled the correct distance") {
    //           CHECK_EQ(dynamics.time() - dynamics.agents().at(0)->spawnTime(), 8);
    //           CHECK_EQ(dynamics.agents().at(0)->freeTime(), dynamics.time());
    //           CHECK_EQ(dynamics.agents().at(0)->streetId().value(), 5);
    //           CHECK_EQ(dynamics.agents().at(0)->speed(), 13.8888888889);
    //           CHECK_EQ(dynamics.agents().at(0)->distance(), 4.);
    //         }
    // #endif
    //       }
    //     }
  }
  //   SUBCASE("streetMeanSpeed") {
  //     /// GIVEN: a dynamics object
  //     /// WHEN: we evolve the dynamics
  //     /// THEN: the agent mean speed is the same as the street mean speed
  //     Road::setMeanVehicleLength(2.);
  //     Street s1{0, std::make_pair(0, 1), 20., 20.};
  //     Street s2{1, std::make_pair(1, 2), 30., 15.};
  //     Street s3{2, std::make_pair(3, 1), 30., 15.};
  //     Street s4{3, std::make_pair(1, 4), 30., 15.};
  //     RoadNetwork graph2;
  //     graph2.addStreets(s1, s2, s3, s4);
  //     graph2.buildAdj();
  //     for (const auto& [nodeId, node] : graph2.nodes()) {
  //       node->setCapacity(4);
  //       node->setTransportCapacity(4);
  //     }
  //     FirstOrderDynamics dynamics{graph2, false, 69, 0.5};
  //     dynamics.addItinerary(2, 2);
  //     dynamics.updatePaths();
  //     dynamics.addAgents(4, 2, 0);
  //     dynamics.evolve(false);
  //     dynamics.evolve(false);
  //     double meanSpeed{0.};
  //     for (const auto& pAgent : dynamics.agents()) {
  //       meanSpeed += pAgent->speed();
  //     }
  //     auto const& pStreet{dynamics.graph().edge(1)};
  //     meanSpeed /= (pStreet->nExitingAgents() + pStreet->movingAgents().size());
  //     CHECK_EQ(dynamics.streetMeanSpeed(1), meanSpeed);
  //     // I don't think the mean speed of agents should be equal to the street's
  //     // one... CHECK_EQ(dynamics.streetMeanSpeed().mean,
  //     // dynamics.agentMeanSpeed().mean); CHECK_EQ(dynamics.streetMeanSpeed().std,
  //     // 0.); street 1 density should be 0.4 so...
  //     CHECK_EQ(dynamics.streetMeanSpeed(0.2, true).mean, meanSpeed);
  //     CHECK_EQ(dynamics.streetMeanSpeed(0.2, true).std, 0.);
  //     CHECK_EQ(dynamics.streetMeanSpeed(0.2, false).mean, 15.);
  //     CHECK_EQ(dynamics.streetMeanSpeed(0.2, false).std, 0.);
  //     dynamics.evolve(false);
  //     meanSpeed = 0.;
  //     for (auto const& pAgent : dynamics.agents()) {
  //       if (!pAgent->streetId().has_value())
  //         continue;
  //       if (pAgent->streetId().value() == 1) {
  //         meanSpeed += pAgent->speed();
  //       }
  //     }
  //     meanSpeed /= pStreet->queue(0).size();
  //     CHECK_EQ(pStreet->queue(0).size(), 3);
  //     CHECK_EQ(dynamics.streetMeanSpeed(1), meanSpeed);
  //   }
  SUBCASE("Intersection priorities") {
    GIVEN("A dynamics object with five nodes and eight streets") {
      RoadNetwork graph2;
      graph2.addNode<Intersection>(0, std::make_pair(0, 0));
      graph2.addNode<Intersection>(1, std::make_pair(-1, 1));   // A
      graph2.addNode<Intersection>(2, std::make_pair(1, 1));    // B
      graph2.addNode<Intersection>(3, std::make_pair(1, -1));   // C
      graph2.addNode<Intersection>(4, std::make_pair(-1, -1));  // D
      graph2.addEdge<Street>(0, std::make_pair(0, 1), 30., 10.);
      graph2.addEdge<Street>(1, std::make_pair(0, 2), 30., 10.);
      graph2.addEdge<Street>(2, std::make_pair(0, 3), 10., 10.);
      graph2.addEdge<Street>(3, std::make_pair(0, 4), 10., 10.);
      graph2.addEdge<Street>(4, std::make_pair(1, 0), 10., 10.);
      graph2.addEdge<Street>(5, std::make_pair(2, 0), 10., 10.);
      graph2.addEdge<Street>(6, std::make_pair(3, 0), 10., 10.);
      graph2.addEdge<Street>(7, std::make_pair(4, 0), 10., 10.);
      graph2.buildAdj();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.graph().node(0)->setCapacity(3);
      dynamics.graph().node(0)->setTransportCapacity(1);
      auto& nodeO{dynamic_cast<Intersection&>(*dynamics.graph().node(0))};
      dynamics.addItinerary(1, 1);
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      WHEN("We add agents and evolve the dynamics") {
        // add an agent in C, D, A
        dynamics.addAgent(2, 4);  // Second
        dynamics.addAgent(2, 3);  // Third
        dynamics.addAgent(2, 1);  // First
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent in A passes first") {
          CHECK_EQ(dynamics.graph().edge(2)->nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 20);
        }
        dynamics.evolve(false);
        THEN("The agent in D passes second") {
          CHECK_EQ(dynamics.graph().edge(2)->nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(2)->nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
      WHEN("We add agents of another itinerary and update the dynamics") {
        dynamics.addAgent(1, 2);
        dynamics.addAgent(1, 3);
        dynamics.addAgent(1, 4);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        dynamics.evolve(false);
        THEN("The agent in D passes first") {
          CHECK_EQ(dynamics.graph().edge(1)->nAgents(), 1);
          CHECK_EQ(nodeO.agents().size(), 2);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 15);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes second") {
          CHECK_EQ(dynamics.graph().edge(1)->nAgents(), 2);
          CHECK_EQ(nodeO.agents().size(), 1);
          CHECK_EQ(nodeO.agents().begin()->second->streetId().value(), 10);
        }
        dynamics.evolve(false);
        THEN("The agent in C passes last") {
          CHECK_EQ(dynamics.graph().edge(1)->nAgents(), 3);
          CHECK(nodeO.agents().empty());
        }
      }
    }
  }
  SUBCASE("meanSpireFlow") {
    GIVEN("A network with a spireStreet and a normal street") {
      RoadNetwork graph2;
      graph2.addEdge<SpireStreet>(0, std::make_pair(0, 1), 10., 5.);
      graph2.addEdge<Street>(1, std::make_pair(1, 2), 10., 10.);
      graph2.buildAdj();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(std::unique_ptr<Itinerary>(new Itinerary(2, 2)));
      dynamics.updatePaths();
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        auto meanSpireFlow = dynamics.meanSpireInputFlow();
        THEN("The mean flow of the spire street is the same as the agent flow") {
          CHECK_EQ(meanSpireFlow.mean, 0.5);
          CHECK_EQ(meanSpireFlow.std, 0);
        }
        dynamics.evolve(false);
        dynamics.evolve(false);
        meanSpireFlow = dynamics.meanSpireOutputFlow();
        THEN("The mean flow of the spire street is the same as the agent flow") {
          CHECK_EQ(meanSpireFlow.mean, 0.5);
          CHECK_EQ(meanSpireFlow.std, 0);
        }
      }
    }
  }
  SUBCASE("meanSpireFlow") {
    GIVEN("A network with a spireStreet and a normal street") {
      RoadNetwork graph2;
      graph2.addEdge<SpireStreet>(0, std::make_pair(0, 1), 10., 5.);
      graph2.addEdge(1, std::make_pair(1, 2), 10., 10.);
      graph2.buildAdj();
      FirstOrderDynamics dynamics{
          graph2, false, 69, 0., dsm::weight_functions::streetLength, 1.};
      dynamics.addItinerary(2, 2);
      dynamics.updatePaths();
      dynamics.addAgent(2, 0);
      WHEN("We evolve the dynamics") {
        dynamics.evolve(false);
        dynamics.evolve(false);
        auto meanSpireFlow = dynamics.meanSpireInputFlow();
        THEN("The mean flow of the spire street is the same as the agent flow") {
          CHECK_EQ(meanSpireFlow.mean, 0.5);
          CHECK_EQ(meanSpireFlow.std, 0);
        }
        dynamics.evolve(false);
        dynamics.evolve(false);
        meanSpireFlow = dynamics.meanSpireOutputFlow();
        THEN("The mean flow of the spire street is the same as the agent flow") {
          CHECK_EQ(meanSpireFlow.mean, 0.5);
          CHECK_EQ(meanSpireFlow.std, 0);
        }
      }
    }
  }
}
