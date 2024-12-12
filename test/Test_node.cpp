#include <cstdint>

#include "Node.hpp"
#include "Intersection.hpp"
#include "TrafficLight.hpp"

#include "doctest.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

using Intersection = dsm::Intersection;
using TrafficLight = dsm::TrafficLight;

TEST_CASE("Intersection") {
  SUBCASE("Constructor") {
    GIVEN("Some parameters") {
      constexpr dsm::Id id = 1;
      constexpr double lat = 2.5;
      constexpr double lon = 3.5;
      constexpr dsm::Size capacity = 2;
      constexpr dsm::Size transportCapacity = 3;
      const std::string name = "MyName";
      WHEN("An Intersection is created using only an Id") {
        Intersection intersection{id};
        THEN("Parameters are set correctly") {
          CHECK_EQ(intersection.id(), id);
          CHECK_FALSE(intersection.coords().has_value());
          CHECK_EQ(intersection.capacity(), 1);
          CHECK_EQ(intersection.transportCapacity(), 1);
          CHECK(intersection.name().empty());
        }
      }
      WHEN("An Intersection is created using an Id and coordinates") {
        Intersection intersection{id, std::make_pair(lat, lon)};
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
        Intersection base{id, std::make_pair(lat, lon)};
        base.setCapacity(capacity);
        base.setTransportCapacity(transportCapacity);
        base.setName(name);
        auto copy = base;
        THEN("Parameters are set correctly") {
          CHECK_EQ(copy.id(), id);
          CHECK(copy.coords().has_value());
          CHECK_EQ(copy.coords().value().first, lat);
          CHECK_EQ(copy.coords().value().second, lon);
          CHECK_EQ(copy.capacity(), capacity);
          CHECK_EQ(copy.transportCapacity(), transportCapacity);
          CHECK_EQ(copy.name(), name);
        }
      }
    }
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
      Intersection intersection(0, std::make_pair(1., 2.));
      WHEN("The node is converted to a traffic light") {
        TrafficLight tl(intersection, 60);
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
      TrafficLight tl{0, 2};
      tl.setCycle(0, dsm::Direction::LEFT, {1, 0});
      WHEN("We increase counter") {
        ++tl;
        THEN("The traffic light is green for all except Left and U turns") {
          CHECK(tl.isGreen(0, dsm::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::STRAIGHT));
          CHECK_FALSE(tl.isGreen(0, dsm::Direction::LEFT));
          CHECK_FALSE(tl.isGreen(0, dsm::Direction::UTURN));
        }
        ++tl;
        THEN("The traffic light is green for all") {
          CHECK(tl.isGreen(0, dsm::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::LEFT));
          CHECK(tl.isGreen(0, dsm::Direction::UTURN));
        }
      }
    }
    GIVEN("A traffic light object with a cycle set") {
      TrafficLight tl{0, 3};
      tl.setCycle(0, dsm::Direction::RIGHT, {2, 2});
      THEN("Traffic light is green for all") {
        CHECK(tl.isGreen(0, dsm::Direction::RIGHT));
        CHECK(tl.isGreen(0, dsm::Direction::STRAIGHT));
        CHECK(tl.isGreen(0, dsm::Direction::LEFT));
        CHECK(tl.isGreen(0, dsm::Direction::UTURN));
      }
      WHEN("We increase counter") {
        ++tl;
        THEN("Traffic light is green for all except Right") {
          CHECK_FALSE(tl.isGreen(0, dsm::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::LEFT));
          CHECK(tl.isGreen(0, dsm::Direction::UTURN));
        }
        ++tl;
        THEN("Traffic light is green for all") {
          CHECK(tl.isGreen(0, dsm::Direction::RIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::STRAIGHT));
          CHECK(tl.isGreen(0, dsm::Direction::LEFT));
          CHECK(tl.isGreen(0, dsm::Direction::UTURN));
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
