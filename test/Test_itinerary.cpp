#include <cstdint>

#include "Itinerary.hpp"

#include "doctest.h"

using Itinerary = dsf::Itinerary;

TEST_CASE("Itinerary") {
  SUBCASE("Constructors") {
    GIVEN("Some parameters") {
      dsf::Id itineraryId{0};
      dsf::Id destinationId{2};
      WHEN("The Itinerary is constructed") {
        Itinerary itinerary{itineraryId, destinationId};
        THEN("The source and destination are set correctly") {
          CHECK_EQ(itinerary.id(), itineraryId);
          CHECK_EQ(itinerary.destination(), destinationId);
        }
      }
    }
  }
}
