#include <cstdint>

#include "../src/dsf/base/Node.hpp"
#include "../src/dsf/mobility/Intersection.hpp"
#include "../src/dsf/mobility/TrafficLight.hpp"
// #include "../src/dsf/mobility/Roundabout.hpp"
#include "../src/dsf/mobility/Station.hpp"
#include "../src/dsf/utility/Typedef.hpp"

#include "doctest.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

using Intersection = dsf::mobility::Intersection;
using TrafficLight = dsf::mobility::TrafficLight;
using Station = dsf::mobility::Station;

TEST_CASE("Node basic") {
  SUBCASE("Constructors and getters") {
    dsf::Node n1{5};
    CHECK_EQ(n1.id(), 5);
    CHECK_FALSE(n1.geometry().has_value());
    CHECK_EQ(n1.name(), "");
    CHECK(n1.ingoingEdges().empty());
    CHECK(n1.outgoingEdges().empty());
    CHECK_FALSE(n1.isStation());

    dsf::Node n2{6, dsf::geometry::Point{1.0, 2.0}};
    CHECK_EQ(n2.id(), 6);
    REQUIRE(n2.geometry().has_value());
    CHECK_EQ(n2.geometry()->x(), 1.0);
    CHECK_EQ(n2.geometry()->y(), 2.0);
  }

  SUBCASE("Copy and assignment") {
    dsf::Node n1{7, dsf::geometry::Point{3.0, 4.0}};
    n1.setName("A node");
    n1.addIngoingEdge(10);
    n1.addOutgoingEdge(11);

    dsf::Node n2 = n1;  // copy ctor
    CHECK_EQ(n2.id(), 7);
    REQUIRE(n2.geometry().has_value());
    CHECK_EQ(n2.geometry()->x(), 3.0);
    CHECK_EQ(n2.name(), "A node");
    CHECK_EQ(n2.ingoingEdges().size(), 1);
    CHECK_EQ(n2.outgoingEdges().size(), 1);

    dsf::Node n3{0};
    n3 = n1;  // assignment
    CHECK_EQ(n3.id(), 7);
    CHECK_EQ(n3.name(), "A node");
    CHECK_EQ(n3.ingoingEdges().front(), 10);
    CHECK_EQ(n3.outgoingEdges().front(), 11);
  }

  SUBCASE("Setters and edge additions") {
    dsf::Node n{8};
    n.setId(9);
    CHECK_EQ(n.id(), 9);
    n.setGeometry(dsf::geometry::Point{4.5, -1.2});
    REQUIRE(n.geometry().has_value());
    CHECK_EQ(n.geometry()->x(), 4.5);
    n.setName("Test node");
    CHECK_EQ(n.name(), "Test node");

    n.addIngoingEdge(20);
    CHECK_EQ(n.ingoingEdges().size(), 1);
    CHECK_EQ(n.ingoingEdges().front(), 20);

    n.addOutgoingEdge(30);
    CHECK_EQ(n.outgoingEdges().size(), 1);
    CHECK_EQ(n.outgoingEdges().front(), 30);

    // adding duplicate should throw
    CHECK_THROWS_AS(n.addIngoingEdge(20), std::invalid_argument);
    CHECK_THROWS_AS(n.addOutgoingEdge(30), std::invalid_argument);
  }
}

TEST_CASE("Intersection") {
  SUBCASE("Constructor") {
    constexpr dsf::Id id = 1;
    constexpr double lat = 2.5;
    constexpr double lon = 3.5;
    Intersection intersection{id};
    CHECK_EQ(intersection.id(), id);
    CHECK_FALSE(intersection.geometry().has_value());
    CHECK_EQ(intersection.capacity(), 1);
    CHECK_EQ(intersection.transportCapacity(), 1);
    CHECK(intersection.name().empty());
    Intersection intersection2{id, dsf::geometry::Point{lon, lat}};
    CHECK_EQ(intersection2.id(), id);
    CHECK(intersection2.geometry().has_value());
    CHECK_EQ(intersection2.geometry().value().x(), lon);
    CHECK_EQ(intersection2.geometry().value().y(), lat);
    CHECK_EQ(intersection2.capacity(), 1);
    CHECK_EQ(intersection2.transportCapacity(), 1);
    CHECK(intersection2.name().empty());
  }

  SUBCASE("Agent management and priorities") {
    Intersection intersection{42};
    intersection.setCapacity(3);
    // Add agents with different angles
    auto agent1 = std::make_unique<dsf::mobility::Agent>(1);
    auto agent2 = std::make_unique<dsf::mobility::Agent>(2);
    auto agent3 = std::make_unique<dsf::mobility::Agent>(3);
    intersection.addAgent(10.0, std::move(agent1));
    intersection.addAgent(5.0, std::move(agent2));
    intersection.addAgent(20.0, std::move(agent3));
    CHECK_EQ(intersection.nAgents(), 3);
    CHECK(intersection.isFull());
    // Density calculation
    CHECK_EQ(intersection.density(), doctest::Approx(1.0));
    // Street priorities
    std::set<dsf::Id> priorities{100, 101};
    intersection.addIngoingEdge(100);
    intersection.addIngoingEdge(101);
    intersection.setStreetPriorities(priorities);
    CHECK_EQ(intersection.streetPriorities(), priorities);
    intersection.addStreetPriority(100);
    CHECK(intersection.streetPriorities().count(100));
    // Error on adding non-existent street priority
    CHECK_THROWS_AS(intersection.addStreetPriority(999), std::invalid_argument);
  }

  SUBCASE("Capacity and error handling") {
    Intersection intersection{99};
    intersection.setCapacity(2);
    auto agent1 = std::make_unique<dsf::mobility::Agent>(1);
    auto agent2 = std::make_unique<dsf::mobility::Agent>(2);
    intersection.addAgent(0.0, std::move(agent1));
    intersection.addAgent(1.0, std::move(agent2));
    CHECK(intersection.isFull());
    auto agent3 = std::make_unique<dsf::mobility::Agent>(3);
    CHECK_THROWS_AS(intersection.addAgent(2.0, std::move(agent3)), std::runtime_error);
    // Lowering capacity below current agent count throws
    CHECK_THROWS_AS(intersection.setCapacity(1), std::runtime_error);
  }
}

TEST_CASE("TrafficLight") {
  SUBCASE("Constructor") {
    GIVEN("A traffic light object") {
      TrafficLight tl{0, 60};
      THEN("The traffic light is created with the correct id and cycle time") {
        CHECK_EQ(tl.id(), 0);
        CHECK_EQ(tl.cycleTime(), 60);
      }
    }
    GIVEN("A node object") {
      Intersection intersection(0, dsf::geometry::Point{1., 2.});
      WHEN("The node is converted to a traffic light") {
        TrafficLight tl(intersection, 60);
        THEN("The traffic light is created with correct parameters") {
          CHECK_EQ(tl.id(), 0);
          CHECK_EQ(tl.cycleTime(), 60);
          CHECK(tl.geometry().has_value());
          CHECK_EQ(tl.geometry().value().x(), 1.);
          CHECK_EQ(tl.geometry().value().y(), 2.);
        }
      }
    }
  }
  SUBCASE("Light cycle") {
    TrafficLight::setAllowFreeTurns(true);
    GIVEN("A traffic light object with a cycle set") {
      TrafficLight tl{0, 2};
      tl.setCycle(0, dsf::Direction::LEFT, {1, 0});
      WHEN("We increase counter") {
        ++tl;
        THEN("The traffic light is green for all except Left and U turns") {
          CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK_FALSE(tl.isGreen(0, dsf::Direction::LEFT));
          CHECK_FALSE(tl.isGreen(0, dsf::Direction::UTURN));
        }
        ++tl;
        THEN("The traffic light is green for all") {
          CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::LEFT));
          CHECK(tl.isGreen(0, dsf::Direction::UTURN));
        }
      }
    }
    GIVEN("A traffic light object with a cycle set") {
      TrafficLight tl{0, 3};
      tl.setCycle(0, dsf::Direction::RIGHT, {2, 2});
      THEN("Traffic light is green for all") {
        CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
        CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
        CHECK(tl.isGreen(0, dsf::Direction::LEFT));
        CHECK(tl.isGreen(0, dsf::Direction::UTURN));
      }
      WHEN("We increase counter") {
        ++tl;
        THEN("Traffic light is green for all except Right") {
          CHECK_FALSE(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::LEFT));
          CHECK(tl.isGreen(0, dsf::Direction::UTURN));
        }
        ++tl;
        THEN("Traffic light is green for all") {
          CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::LEFT));
          CHECK(tl.isGreen(0, dsf::Direction::UTURN));
        }
      }
    }
  }
  // SUBCASE("Phase") {
  //   /// This tests the phase.
  //   /// GIVEN: A TrafficLight
  //   /// WHEN: The phase is set to update after a green-red cycle
  //   /// THEN: It's checked that the current gree-red cycle is not affected
  //   /// and ultimately it's checked that on the next green-red cycle the phase is updated correctly
  //   TrafficLight trafficLight{0};
  //   trafficLight.setDelay(std::make_pair(5, 7));
  //   trafficLight.setPhaseAfterCycle(6);

  //   for (size_t i = 0; i < 12; ++i) {
  //     trafficLight.increaseCounter();
  //   }
  //   CHECK_FALSE(trafficLight.isGreen());
  //   trafficLight.increaseCounter();
  //   CHECK_FALSE(trafficLight.isGreen());

  //   trafficLight.setPhase(0);
  //   CHECK(trafficLight.isGreen());

  //   for (size_t i = 0; i < 12; ++i) {
  //     trafficLight.increaseCounter();
  //   }
  //   CHECK(trafficLight.isGreen());
  // }
  // SUBCASE("Asymmetric traffic light") {
  //   /// This tests the asymmetric traffic light.
  //   /// GIVEN: A TrafficLight
  //   /// WHEN: The asymmetric traffic light is set
  //   /// THEN: The asymmetric traffic light is set correctly
  //   TrafficLight trafficLight{0};
  //   trafficLight.setDelay(std::make_pair(5, 3));
  //   for (size_t i = 0; i < 8; ++i) {
  //     if (i < 5) {
  //       CHECK(trafficLight.isGreen());
  //     } else {
  //       CHECK_FALSE(trafficLight.isGreen());
  //     }
  //     trafficLight.increaseCounter();
  //   }
  //   CHECK(trafficLight.isGreen());
  // }
  // SUBCASE("Dynamic traffic light") {
  //   GIVEN("A traffic ligth object with set delay") {
  //     TrafficLight tl{0};
  //     tl.setDelay(std::make_pair(5, 3));
  //     WHEN("The delay is set with a green value smaller than the previous one") {
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.setDelay(std::make_pair(2, 3));
  //       THEN("It is green for two cycles") {
  //         CHECK(tl.isGreen());
  //         tl.increaseCounter();
  //         CHECK(tl.isGreen());
  //         tl.increaseCounter();
  //         CHECK_FALSE(tl.isGreen());
  //       }
  //     }
  //     WHEN("The delay is set with a red value smaller than the previous one") {
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.increaseCounter();
  //       tl.setDelay(std::make_pair(1, 3));
  //       THEN("It is red for one cycles") {
  //         CHECK_FALSE(tl.isGreen());
  //         tl.increaseCounter();
  //         CHECK(tl.isGreen());
  //       }
  //     }
  //   }
  // }
}

TEST_CASE("Station") {
  SUBCASE("Constructors") {
    constexpr dsf::Id id = 1;
    constexpr dsf::Delay managementTime = 2;
    constexpr double lat = 2.5;
    constexpr double lon = 3.5;
    const std::string name = "S0001";
    GIVEN("A Station object") {
      WHEN("The Station is created using only an Id") {
        Station station{id, managementTime};
        THEN("Parameters are set correctly") {
          CHECK_EQ(station.id(), id);
          CHECK_EQ(station.managementTime(), managementTime);
          CHECK_EQ(station.capacity(), 1);
          CHECK_EQ(station.transportCapacity(), 1);
          CHECK(station.name().empty());
        }
      }
      WHEN("The Station is created using an Id and coordinates") {
        Station station{id, dsf::geometry::Point(lat, lon), managementTime};
        THEN("Parameters are set correctly") {
          CHECK_EQ(station.id(), id);
          CHECK_EQ(station.managementTime(), managementTime);
          CHECK_EQ(station.capacity(), 1);
          CHECK_EQ(station.transportCapacity(), 1);
          CHECK(station.name().empty());
        }
      }
      WHEN("The Station is created using a copy constructor") {
        Station base{id, dsf::geometry::Point(lat, lon), managementTime};
        base.setCapacity(2);
        base.setTransportCapacity(3);
        base.setName(name);
        auto copy = base;
        THEN("Parameters are set correctly") {
          CHECK_EQ(copy.id(), id);
          CHECK_EQ(copy.managementTime(), managementTime);
          CHECK_EQ(copy.capacity(), 2);
          CHECK_EQ(copy.transportCapacity(), 3);
          CHECK_EQ(copy.name(), name);
        }
      }
    }
  }
  SUBCASE("Enqueue and dequeue") {
    constexpr dsf::Id id = 1;
    constexpr dsf::Delay managementTime = 2;
    GIVEN("A Station object") {
      Station station{id, managementTime};
      WHEN("A train is enqueued") {
        station.enqueue(1, dsf::train_t::BUS);
        THEN("The train is enqueued correctly") { CHECK_EQ(station.dequeue(), 1); }
      }
      WHEN("Multiple trains are enqueued") {
        station.enqueue(1, dsf::train_t::RV);
        station.enqueue(2, dsf::train_t::FRECCIAROSSA);
        THEN("The trains are enqueued correctly") {
          CHECK_EQ(station.dequeue(), 2);
          CHECK_EQ(station.dequeue(), 1);
        }
      }
    }
  }
}