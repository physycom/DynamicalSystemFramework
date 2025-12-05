#include <cstdint>
#include <format>
#include <string>

#include "dsf/mobility/Agent.hpp"
#include "dsf/mobility/Itinerary.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using Agent = dsf::mobility::Agent;

TEST_CASE("Agent") {
  SUBCASE("Constructors") {
    GIVEN("An agent and its itinerary ids") {
      uint16_t itineraryId{0};
      WHEN("The Agent is constructed") {
        Agent agent{0, itineraryId};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.itineraryId(), itineraryId);
          CHECK_FALSE(agent.streetId().has_value());
          CHECK_FALSE(agent.srcNodeId().has_value());
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("An agent, its itinerary ids, and a node id") {
      uint16_t itineraryId{0};
      uint16_t srcNodeId{0};
      WHEN("The Agent is constructed") {
        Agent agent{0, itineraryId, srcNodeId};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.itineraryId(), itineraryId);
          CHECK_FALSE(agent.streetId().has_value());
          CHECK(agent.srcNodeId().has_value());
          CHECK_EQ(agent.srcNodeId().value(), srcNodeId);
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.freeTime(), 0);
          CHECK_EQ(agent.spawnTime(), 0);
        }
      }
    }
    GIVEN("No initinerary ids and no source node id") {
      WHEN("The agent is constructed") {
        auto randomAgent = Agent{0};
        THEN("The agent is a random agent") { CHECK(randomAgent.isRandom()); }
      }
    }
  }
}

TEST_CASE("Agent methods") {
  Agent agent{0, 42, 7};
  SUBCASE("setSrcNodeId and srcNodeId") {
    agent.setSrcNodeId(99);
    CHECK(agent.srcNodeId().has_value());
    CHECK_EQ(agent.srcNodeId().value(), 99);
  }
  SUBCASE("setNextStreetId and nextStreetId") {
    agent.setNextStreetId(123);
    CHECK(agent.nextStreetId().has_value());
    CHECK_EQ(agent.nextStreetId().value(), 123);
  }
  SUBCASE("setStreetId with value") {
    agent.setNextStreetId(55);
    CHECK_THROWS_AS(agent.setStreetId(55), std::logic_error);
    CHECK_FALSE(agent.streetId().has_value());
    agent.setStreetId();
    CHECK_EQ(agent.streetId().value(), 55);
    CHECK_FALSE(agent.nextStreetId().has_value());
  }
  SUBCASE("setStreetId with nullopt uses nextStreetId") {
    agent.setNextStreetId(77);
    agent.setStreetId();
    CHECK(agent.streetId().has_value());
    CHECK_EQ(agent.streetId().value(), 77);
  }
  SUBCASE("setSpeed and speed") {
    agent.setSpeed(12.5);
    CHECK_EQ(agent.speed(), 12.5);
  }
  SUBCASE("setFreeTime and freeTime") {
    agent.setFreeTime(1234);
    CHECK_EQ(agent.freeTime(), 1234);
  }
  SUBCASE("incrementDistance and distance") {
    double d0 = agent.distance();
    agent.incrementDistance(5.5);
    CHECK_EQ(agent.distance(), d0 + 5.5);
  }
  SUBCASE("updateItinerary advances index") {
    std::vector<dsf::Id> trip = {1, 2, 3};
    Agent a2{0, trip, 0};
    CHECK_EQ(a2.itineraryId(), 1);
    a2.updateItinerary();
    CHECK_EQ(a2.itineraryId(), 2);
    a2.updateItinerary();
    CHECK_EQ(a2.itineraryId(), 3);
    a2.updateItinerary();  // Should not go out of bounds
    CHECK_EQ(a2.itineraryId(), 3);
  }
  SUBCASE("reset resets state") {
    agent.setSpeed(10.);
    agent.incrementDistance(5.);
    agent.setFreeTime(99);
    agent.setStreetId(88);
    agent.updateItinerary();
    agent.reset(555);
    CHECK_EQ(agent.spawnTime(), 555);
    CHECK_EQ(agent.freeTime(), 0);
    CHECK_EQ(agent.speed(), 0.);
    CHECK_EQ(agent.distance(), 0.);
    CHECK_EQ(agent.itineraryId(), 42);  // itinerary index reset
    CHECK_FALSE(agent.streetId().has_value());
    CHECK_EQ(agent.trip().size(), 1);
  }
}

TEST_CASE("Agent formatting") {
  SUBCASE("std::format with complete agent") {
    Agent agent{0, 42, 7};
    agent.setStreetId(10);
    agent.setNextStreetId(15);
    agent.setSpeed(13.5);
    agent.incrementDistance(100.25);
    agent.setFreeTime(123);

    std::string formatted = std::format("{}", agent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: 10") != std::string::npos);
    CHECK(formatted.find("srcNodeId: 7") != std::string::npos);
    CHECK(formatted.find("nextStreetId: 15") != std::string::npos);
    CHECK(formatted.find("itineraryId: 42") != std::string::npos);
    CHECK(formatted.find("speed: 13.50 m/s") != std::string::npos);
    CHECK(formatted.find("distance: 100.25 m") != std::string::npos);
    CHECK(formatted.find("spawnTime: 0") != std::string::npos);
    CHECK(formatted.find("freeTime: 123") != std::string::npos);
  }

  SUBCASE("std::format with random agent") {
    Agent randomAgent{5};

    std::string formatted = std::format("{}", randomAgent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: N/A") != std::string::npos);
    CHECK(formatted.find("srcNodeId: N/A") != std::string::npos);
    CHECK(formatted.find("nextStreetId: N/A") != std::string::npos);
    CHECK(formatted.find("itineraryId: RANDOM") != std::string::npos);
    CHECK(formatted.find("speed: 0.00 m/s") != std::string::npos);
    CHECK(formatted.find("distance: 0.00 m") != std::string::npos);
    CHECK(formatted.find("spawnTime: 5") != std::string::npos);
    CHECK(formatted.find("freeTime: 0") != std::string::npos);
  }

  SUBCASE("std::format with agent with optional nullopts") {
    Agent agent{10, 99};

    std::string formatted = std::format("{}", agent);
    CHECK(formatted.find("id: 0") != std::string::npos);
    CHECK(formatted.find("streetId: N/A") != std::string::npos);
    CHECK(formatted.find("srcNodeId: N/A") != std::string::npos);
    CHECK(formatted.find("nextStreetId: N/A") != std::string::npos);
    CHECK(formatted.find("itineraryId: 99") != std::string::npos);
  }
  SUBCASE("hasArrived") {
    Agent agent{10};  // spawnTime = 10
    CHECK(agent.isRandom());

    // Test Max Distance
    agent.setMaxDistance(100.0);
    CHECK_FALSE(agent.hasArrived(std::nullopt));

    agent.incrementDistance(99.0);
    CHECK_FALSE(agent.hasArrived(std::nullopt));

    agent.incrementDistance(1.0);
    CHECK(agent.hasArrived(std::nullopt));

    // Test Max Time
    // Reset agent
    agent = Agent{10};
    agent.setMaxTime(50);  // Max duration 50. Should expire at 10+50=60.

    // Before expiration
    CHECK_FALSE(agent.hasArrived(59));
    // At expiration
    CHECK(agent.hasArrived(60));
    // After expiration
    CHECK(agent.hasArrived(61));
  }
}
