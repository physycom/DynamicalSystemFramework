#include <cstdint>

#include "Itinerary.hpp"

#include "doctest.h"

using Itinerary = dsm::Itinerary;

TEST_CASE("Itinerary") {
  SUBCASE("Constructors") {
    GIVEN("An itinerary and its destination ids") {
      uint8_t destinationId{2};
      WHEN("The Itinerary is constructed") {
        Itinerary itinerary{destinationId};
        THEN("The destination is set correctly") {
          CHECK_EQ(itinerary.destination(), destinationId);
        }
      }
    }
    GIVEN("An itinerary id, its destination id and a transition matrix") {
      uint8_t destinationId{2};
      dsm::SparseMatrix<bool> path{1, 1};
      WHEN("The Itinerary is constructed") {
        Itinerary itinerary{destinationId, path};
        THEN("The destination and path are set correctly") {
          CHECK_EQ(itinerary.destination(), destinationId);
          CHECK_EQ(itinerary.path().getRowDim(), 1);
          CHECK_EQ(itinerary.path().getColDim(), 1);
        }
      }
    }
  }
}
