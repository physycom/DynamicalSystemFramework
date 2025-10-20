#include <cstdint>

#include "Node.hpp"
#include "Intersection.hpp"
#include "TrafficLight.hpp"
#include "Station.hpp"
#include "../utility/Typedef.hpp"

#include "doctest.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

TEST_CASE("Intersection") {
  SUBCASE("Constructor") {
    GIVEN("Some parameters") {
      constexpr dsf::Id id = 1;
      constexpr double lat = 2.5;
      constexpr double lon = 3.5;
      const std::string name = "MyName";
      WHEN("An Intersection is created using only an Id") {
        dsf::Intersection intersection{id};
        THEN("Parameters are set correctly") {
          CHECK_EQ(intersection.id(), id);
          CHECK_FALSE(intersection.coords().has_value());
          CHECK_EQ(intersection.capacity(), 1);
          CHECK_EQ(intersection.transportCapacity(), 1);
          CHECK(intersection.name().empty());
        }
      }
      WHEN("An Intersection is created using an Id and coordinates") {
        dsf::Intersection intersection{id, std::make_pair(lat, lon)};
        THEN("Parameters are set correctly") {
          CHECK_EQ(intersection.id(), id);
          CHECK(intersection.coords().has_value());
          CHECK_EQ(intersection.coords().value().first, lat);
          CHECK_EQ(intersection.coords().value().second, lon);
          CHECK_EQ(intersection.capacity(), 1);
          CHECK_EQ(intersection.transportCapacity(), 1);
          CHECK(intersection.name().empty());
        }
      }
      WHEN("An intersection is created using copy constructor") {
        // Intersection base{id, std::make_pair(lat, lon)};
        // base.setCapacity(capacity);
        // base.setTransportCapacity(transportCapacity);
        // base.setName(name);
        // auto copy = base;
        // THEN("Parameters are set correctly") {
        //   CHECK_EQ(copy.id(), id);
        //   CHECK(copy.coords().has_value());
        //   CHECK_EQ(copy.coords().value().first, lat);
        //   CHECK_EQ(copy.coords().value().second, lon);
        //   CHECK_EQ(copy.capacity(), capacity);
        //   CHECK_EQ(copy.transportCapacity(), transportCapacity);
        //   CHECK_EQ(copy.name(), name);
        // }
      }
    }
  }
}

TEST_CASE("TrafficLight") {
  SUBCASE("Constructor") {
    GIVEN("A traffic light object") {
      dsf::TrafficLight tl{0, 60};
      THEN("The traffic light is created with the correct id and cycle time") {
        CHECK_EQ(tl.id(), 0);
        CHECK_EQ(tl.cycleTime(), 60);
      }
    }
    GIVEN("A node object") {
      dsf::Intersection intersection(0, std::make_pair(1., 2.));
      WHEN("The node is converted to a traffic light") {
        dsf::TrafficLight tl(intersection, 60);
        THEN("The traffic light is created with correct parameters") {
          CHECK_EQ(tl.id(), 0);
          CHECK_EQ(tl.cycleTime(), 60);
          CHECK(tl.coords().has_value());
          CHECK_EQ(tl.coords().value().first, 1.);
          CHECK_EQ(tl.coords().value().second, 2.);
        }
      }
    }
  }
  SUBCASE("Light cycle") {
    GIVEN("A traffic light object with a cycle set") {
      dsf::TrafficLight tl{0, 20};
      WHEN("We create a TrafficLightPhase") {
        dsf::TrafficLightPhase phase{10, dsf::Direction::ANY};
        THEN("We have to insert at least one street id") {
          CHECK_THROWS_AS(tl.setPhases({phase}), std::invalid_argument);
        }
        phase.setStreetIds({0});
        THEN("The inserted streetId must be an ingoing edge of the traffic light") {
          CHECK_THROWS_AS(tl.setPhases({phase}), std::invalid_argument);
        }
        tl.addIngoingEdge(0);
        tl.setPhases({phase});
        THEN("The phase is set correctly") {
          CHECK_EQ(tl.phases().size(), 1);
          CHECK_EQ(tl.phases().front().greenTime(), 10);
          CHECK_EQ(tl.phases().front().direction(), dsf::Direction::ANY);
          CHECK_EQ(tl.phases().front().streetIds().size(), 1);
          CHECK_EQ(*(tl.phases().front().streetIds().begin()), 0);
        }
        dsf::TrafficLightPhase otherphase{15, dsf::Direction::RIGHT};
        otherphase.setStreetIds({0});
        THEN("It must have the green time not exceeding the cycle time") {
          CHECK_THROWS_AS(tl.setPhases({phase, otherphase}), std::invalid_argument);
        }
      }
    }
    GIVEN("A traffic light object with a cycle set") {
      dsf::TrafficLight tl{0, 3};
      tl.addIngoingEdge(0);
      tl.addFreeTurn(0, dsf::Direction::LEFTANDSTRAIGHT);
      dsf::TrafficLightPhase phase1{2, dsf::Direction::RIGHT};
      phase1.setStreetIds({0});
      tl.setPhases({phase1});
      THEN("Traffic light is green for all") {
        CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
        CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
        CHECK(tl.isGreen(0, dsf::Direction::LEFT));
      }
      WHEN("We increase counter") {
        ++tl;
        THEN("Traffic light is green for all") {
          CHECK(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::LEFT));
        }
        ++tl;
        THEN("Traffic light is green for all except RIGHT") {
          CHECK_FALSE(tl.isGreen(0, dsf::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsf::Direction::LEFT));
        }
      }
    }
  }
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
        dsf::Station station{id, managementTime};
        THEN("Parameters are set correctly") {
          CHECK_EQ(station.id(), id);
          CHECK_EQ(station.managementTime(), managementTime);
          CHECK_EQ(station.capacity(), 1);
          CHECK_EQ(station.transportCapacity(), 1);
          CHECK(station.name().empty());
        }
      }
      WHEN("The Station is created using an Id and coordinates") {
        dsf::Station station{id, std::make_pair(lat, lon), managementTime};
        THEN("Parameters are set correctly") {
          CHECK_EQ(station.id(), id);
          CHECK_EQ(station.managementTime(), managementTime);
          CHECK_EQ(station.capacity(), 1);
          CHECK_EQ(station.transportCapacity(), 1);
          CHECK(station.name().empty());
        }
      }
      WHEN("The Station is created using a copy constructor") {
        dsf::Station base{id, std::make_pair(lat, lon), managementTime};
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
      dsf::Station station{id, managementTime};
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