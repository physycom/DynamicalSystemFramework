#include <cstdint>

#include "Agent.hpp"
#include "Itinerary.hpp"

#include "doctest.h"

using Agent = dsm::Agent<uint16_t, uint16_t, uint16_t>;
using Itinerary = dsm::Itinerary<uint16_t>;

TEST_CASE("Agent") {
  SUBCASE("Constructors") {
    GIVEN("An agent and its itinerary ids") {
      uint16_t agentId{1};
      uint16_t itineraryId{0};
      WHEN("The Agent is constructed") {
        Agent agent{agentId, itineraryId};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.id(), agentId);
          CHECK_EQ(agent.itineraryId(), itineraryId);
          CHECK_FALSE(agent.streetId().has_value());
          CHECK_FALSE(agent.srcNodeId().has_value());
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.delay(), 0);
          CHECK_EQ(agent.time(), 0);
        }
      }
    }
    GIVEN("An agent, its itinerary ids, and a node id") {
      uint16_t agentId{1};
      uint16_t itineraryId{0};
      uint16_t srcNodeId{0};
      WHEN("The Agent is constructed") {
        Agent agent{agentId, itineraryId, srcNodeId};
        THEN("The agent and itinerary ids are set correctly") {
          CHECK_EQ(agent.id(), agentId);
          CHECK_EQ(agent.itineraryId(), itineraryId);
          CHECK_FALSE(agent.streetId().has_value());
          CHECK(agent.srcNodeId().has_value());
          CHECK_EQ(agent.srcNodeId().value(), srcNodeId);
          CHECK_EQ(agent.speed(), 0);
          CHECK_EQ(agent.delay(), 0);
          CHECK_EQ(agent.time(), 0);
        }
      }
    }
  }
}
