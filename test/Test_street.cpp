#include <cstdint>
#include <optional>
#include <numbers>

#include "Agent.hpp"
#include "Intersection.hpp"
#include "Street.hpp"
#include "../utility/Typedef.hpp"

#include "doctest.h"

using Agent = dsm::Agent<double>;
using Intersection = dsm::Intersection;
using Street = dsm::Street;
using Road = dsm::Road;
using SpireStreet = dsm::SpireStreet;

TEST_CASE("Street") {
  Road::setMeanVehicleLength(1.);
  SUBCASE("Constructor_1") {
    /*This tests the constructor that takes an Id, capacity, and length.
    GIVEN: An Id, capacity, and length
    WHEN: A Street is constructed
    THEN: The Id, capacity, and length are set correctly
    */

    Street street{1, std::make_pair(0, 1)};
    CHECK_EQ(street.id(), 1);
    CHECK_EQ(street.capacity(), 1);
    CHECK_EQ(street.transportCapacity(), 1);
    CHECK_EQ(street.length(), 1.);
    CHECK_EQ(street.nodePair().first, 0);
    CHECK_EQ(street.nodePair().second, 1);
    CHECK_EQ(street.maxSpeed(), 13.8888888889);
    CHECK_EQ(street.nLanes(), 1);
  }

  SUBCASE("Constructor_2") {
    /*This tests the constructor that takes an Id, capacity, length, and nodePair.
    GIVEN: An Id, capacity, length and nodePair
    WHEN: A Street is constructed
    THEN: The Id, capacity, length and nodePair are set correctly
    */

    Street street{1, std::make_pair(4, 5), 1.5};
    CHECK_EQ(street.id(), 1);
    CHECK_EQ(street.capacity(), 2);
    CHECK_EQ(street.transportCapacity(), 1);
    CHECK_EQ(street.length(), 1.5);
    CHECK_EQ(street.source(), 4);
    CHECK_EQ(street.target(), 5);
    CHECK_EQ(doctest::Approx(street.density()), 0);
    CHECK_EQ(street.maxSpeed(), 13.8888888889);
    CHECK_EQ(street.nLanes(), 1);
  }
  SUBCASE("Constructor_3") {
    /*This tests the constructor that takes an Id, capacity, length, nodePair, and maxSpeed.
    GIVEN: An Id, capacity, length, nodePair, and maxSpeed
    WHEN: A Street is constructed
    THEN: The Id, capacity, length, nodePair, and maxSpeed are set correctly
    */

    Street street{1, std::make_pair(4, 5), 1.5, 40.};
    CHECK_EQ(street.id(), 1);
    CHECK_EQ(street.capacity(), 2);
    CHECK_EQ(street.transportCapacity(), 1);
    CHECK_EQ(street.length(), 1.5);
    CHECK_EQ(street.source(), 4);
    CHECK_EQ(street.target(), 5);
    CHECK_EQ(doctest::Approx(street.density()), 0);
    CHECK_EQ(street.maxSpeed(), 40.);
    CHECK_EQ(street.nLanes(), 1);
  }

  SUBCASE("Enqueue") {
    /*This tests the insertion of an agent in a street's queue*/

    // define some agents
    Agent a1{1, 1, 0};  // they are all in street 1
    Agent a2{2, 1, 0};
    Agent a3{3, 1, 0};
    Agent a4{4, 1, 0};

    Street street{1, std::make_pair(0, 1), 3.5};
    // fill the queue
    street.addAgent(a1.id());
    street.enqueue(a1.id(), 0);
    street.addAgent(a2.id());
    street.enqueue(a2.id(), 0);
    CHECK_EQ(doctest::Approx(street.density()), 0.571429);
    street.addAgent(a3.id());
    street.enqueue(a3.id(), 0);
    street.addAgent(a4.id());
    street.enqueue(a4.id(), 0);
    CHECK_EQ(street.queue(0).front(), 1);
    CHECK_EQ(street.queue(0).back(), 4);
    CHECK_EQ(street.queue(0).size(), street.capacity());
    CHECK_EQ(street.queue(0).size(), street.capacity());
    CHECK_EQ(doctest::Approx(street.density()), 1.14286);
    CHECK(street.isFull());
  }

  SUBCASE("Dequeue") {
    /*This tests the exit of an agent from a street's queue*/

    // define some agents
    Agent a1{1, 1, 0};  // they are all in street 1
    Agent a2{2, 1, 0};
    Agent a3{3, 1, 0};
    Agent a4{4, 1, 0};

    Street street{1, std::make_pair(0, 1), 3.5};
    // fill the queue
    street.addAgent(a1.id());
    street.enqueue(a1.id(), 0);
    street.addAgent(a2.id());
    street.enqueue(a2.id(), 0);
    street.addAgent(a3.id());
    street.enqueue(a3.id(), 0);
    street.addAgent(a4.id());
    street.enqueue(a4.id(), 0);
    CHECK_EQ(street.queue(0).front(),
             1);  // check that agent 1 is at the front of the queue
    // dequeue
    street.dequeue(0);
    CHECK_EQ(street.queue(0).front(), 2);  // check that agent 2 is now at front
    // check that the length of the queue has decreased
    CHECK_EQ(street.queue(0).size(), 3);
    CHECK_EQ(street.queue(0).size(), 3);
    // check that the next agent dequeued is agent 2
    CHECK_EQ(street.dequeue(0), 2);
    CHECK_EQ(street.queue(0).size(), 2);
    street.dequeue(0);
    street.dequeue(0);  // the queue is now empty
  }
  SUBCASE("Angle") {
    /// This tests the angle method
    /// GIVEN: A street
    /// WHEN: The angle method is called
    /// THEN: The angle is returned and is correct
    Street street{1, std::make_pair(0, 1), 3.5};
    CHECK_EQ(street.angle(), 0);
    street.setAngle(std::make_pair(0, 1), std::make_pair(1, 0));
    CHECK_EQ(street.angle(), 3 * std::numbers::pi / 4);
  }
}

TEST_CASE("SpireStreet") {
  SUBCASE("Input flow") {
    GIVEN("A spire street") {
      SpireStreet street{1, std::make_pair(0, 1), 3.5};
      WHEN("An agent is enqueued") {
        street.addAgent(1);
        THEN("The input flow is one") { CHECK_EQ(street.inputCounts(), 1); }
        street.enqueue(1, 0);
        THEN("The density is updated") {
          CHECK_EQ(doctest::Approx(street.density()), 0.285714);
        }
        THEN("Output flow is zero") { CHECK_EQ(street.outputCounts(), 0); }
        THEN("Mean flow is one") { CHECK_EQ(street.meanFlow(), 1); }
      }
      WHEN("Three agents are enqueued") {
        street.addAgent(1);
        street.enqueue(1, 0);
        street.addAgent(2);
        street.enqueue(2, 0);
        street.addAgent(3);
        street.enqueue(3, 0);
        THEN("The density is updated") {
          CHECK_EQ(doctest::Approx(street.density()), 0.857143);
        }
        THEN("Input flow is three") { CHECK_EQ(street.inputCounts(), 3); }
        THEN("Output flow is zero") { CHECK_EQ(street.outputCounts(), 0); }
        THEN("Mean flow is three") { CHECK_EQ(street.meanFlow(), 3); }
      }
      WHEN("An agent is dequeued") {
        street.addAgent(1);
        street.enqueue(1, 0);
        street.dequeue(0);
        THEN("The density is updated") { CHECK_EQ(doctest::Approx(street.density()), 0); }
        THEN("Input flow is one") { CHECK_EQ(street.inputCounts(), 1); }
        THEN("Output flow is one") { CHECK_EQ(street.outputCounts(), 1); }
        THEN("Mean flow is zero") { CHECK_EQ(street.meanFlow(), 0); }
      }
      WHEN("Three agents are dequeued") {
        street.addAgent(1);
        street.enqueue(1, 0);
        street.addAgent(2);
        street.enqueue(2, 0);
        street.addAgent(3);
        street.enqueue(3, 0);
        street.dequeue(0);
        street.dequeue(0);
        street.dequeue(0);
        THEN("The density is updated") { CHECK_EQ(doctest::Approx(street.density()), 0); }
        THEN("Input flow is three") { CHECK_EQ(street.inputCounts(), 3); }
        THEN("Output flow is three") { CHECK_EQ(street.outputCounts(), 3); }
        THEN("Mean flow is zero") { CHECK_EQ(street.meanFlow(), 0); }
      }
      WHEN("Input is greater than output") {
        street.addAgent(1);
        street.enqueue(1, 0);
        street.addAgent(2);
        street.enqueue(2, 0);
        street.dequeue(0);
        street.dequeue(0);
        street.addAgent(3);
        street.enqueue(3, 0);
        THEN("The density is updated") {
          CHECK_EQ(doctest::Approx(street.density()), 0.285714);
        }
        THEN("Mean flow is one") { CHECK_EQ(street.meanFlow(), 1); }
      }
      WHEN("Output is greater than input") {
        street.addAgent(1);
        street.enqueue(1, 0);
        street.addAgent(2);
        street.enqueue(2, 0);
        street.meanFlow();
        street.addAgent(3);
        street.enqueue(3, 0);
        street.dequeue(0);
        street.dequeue(0);
        THEN("The density is updated") {
          CHECK_EQ(doctest::Approx(street.density()), 0.285714);
        }
        THEN("Mean flow is minus one") { CHECK_EQ(street.meanFlow(), -1); }
      }
    }
  }
}
