#include <cstdint>
#include <optional>
#include <numbers>
#include <format>
#include <string>

#include "dsf/mobility/Agent.hpp"
#include "dsf/mobility/Intersection.hpp"
#include "dsf/mobility/Street.hpp"
#include "dsf/utility/Typedef.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using Agent = dsf::mobility::Agent;
using Intersection = dsf::mobility::Intersection;
using Street = dsf::mobility::Street;
using Road = dsf::mobility::Road;

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
  SUBCASE("Lane Mapping") {
    GIVEN("A street with three lanes") {
      Street street{0, std::make_pair(0, 1), 5., 13.8888888889, 3};
      CHECK_EQ(street.laneMapping().size(), 3);
      CHECK_EQ(street.laneMapping()[0], dsf::Direction::RIGHTANDSTRAIGHT);
      CHECK_EQ(street.laneMapping()[1], dsf::Direction::STRAIGHT);
      CHECK_EQ(street.laneMapping()[2], dsf::Direction::LEFT);
      WHEN("We change the lane mapping") {
        street.setLaneMapping(std::vector<dsf::Direction>{
            dsf::Direction::RIGHT, dsf::Direction::STRAIGHT, dsf::Direction::STRAIGHT});
        THEN("The lane mapping is updated") {
          CHECK_EQ(street.laneMapping().size(), 3);
          CHECK_EQ(street.laneMapping()[0], dsf::Direction::RIGHT);
          CHECK_EQ(street.laneMapping()[1], dsf::Direction::STRAIGHT);
          CHECK_EQ(street.laneMapping()[2], dsf::Direction::STRAIGHT);
        }
      }
    }
  }
  SUBCASE("addAgent") {
    Agent a1{0, 1, 0};
    a1.setFreeTime(5);
    Agent a2{0, 1, 0};
    a2.setFreeTime(3);
    Agent a3{0, 1, 0};
    a3.setFreeTime(7);

    Street street{1, std::make_pair(0, 1), 3.5};
    street.addAgent(std::make_unique<Agent>(a1));
    CHECK_EQ(street.movingAgents().top()->freeTime(), 5);
    street.addAgent(std::make_unique<Agent>(a2));
    CHECK_EQ(street.movingAgents().top()->freeTime(), 3);
    street.addAgent(std::make_unique<Agent>(a3));
    CHECK_EQ(street.movingAgents().top()->freeTime(), 3);
  }
  SUBCASE("Enqueue") {
    /*This tests the insertion of an agent in a street's queue*/

    // define some agents
    Agent a1{0, 1, 0};  // they are all in street 1
    Agent a2{0, 1, 0};
    Agent a3{0, 1, 0};
    Agent a4{0, 1, 0};

    Street street{1, std::make_pair(0, 1), 3.5};
    // fill the queue
    street.addAgent(std::make_unique<Agent>(a1));
    street.enqueue(0);
    street.addAgent(std::make_unique<Agent>(a2));
    street.enqueue(0);
    CHECK_EQ(doctest::Approx(street.density()), 0.571429);
    street.addAgent(std::make_unique<Agent>(a3));
    street.enqueue(0);
    street.addAgent(std::make_unique<Agent>(a4));
    street.enqueue(0);
    CHECK(street.queue(0).front());
    CHECK(street.queue(0).back());
    CHECK_EQ(street.queue(0).size(), street.capacity());
    CHECK_EQ(street.queue(0).size(), street.capacity());
    CHECK_EQ(doctest::Approx(street.density()), 1.14286);
    CHECK(street.isFull());
  }

  SUBCASE("Dequeue") {
    /*This tests the exit of an agent from a street's queue*/

    // define some agents
    Agent a1{0, 1, 0};  // they are all in street 1
    Agent a2{0, 1, 0};
    Agent a3{0, 1, 0};
    Agent a4{0, 1, 0};

    Street street{1, std::make_pair(0, 1), 3.5};
    // fill the queue
    street.addAgent(std::make_unique<Agent>(a1));
    street.enqueue(0);
    street.addAgent(std::make_unique<Agent>(a2));
    street.enqueue(0);
    street.addAgent(std::make_unique<Agent>(a3));
    street.enqueue(0);
    street.addAgent(std::make_unique<Agent>(a4));
    street.enqueue(0);
    CHECK(street.queue(0).front());
    // dequeue
    street.dequeue(0);
    CHECK(street.queue(0).front());  // check that agent 2 is now at front
    // check that the length of the queue has decreased
    CHECK_EQ(street.queue(0).size(), 3);
    CHECK_EQ(street.queue(0).size(), 3);
    // check that the next agent dequeued is agent 2
    CHECK(street.dequeue(0));
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
    street.setGeometry(
        dsf::geometry::PolyLine{dsf::geometry::Point{0, 1}, dsf::geometry::Point{1, 0}});
    CHECK_EQ(street.angle(), 7 * std::numbers::pi / 4);
  }
}

TEST_CASE("Street with a coil") {
  SUBCASE("Counts") {
    GIVEN("A street with a counter") {
      Street street{1, std::make_pair(0, 1), 3.5};
      street.enableCounter();
      CHECK_EQ(street.counterName(), "Coil_1");
      WHEN("An agent is added") {
        street.addAgent(std::make_unique<Agent>(0, 1));
        THEN("The input flow is zero") { CHECK_EQ(street.counts(), 0); }
        street.enqueue(0);
        THEN("The input flow is one once enqueued") { CHECK_EQ(street.counts(), 1); }
        street.resetCounter();
        THEN("The counts are 0 after reset") { CHECK_EQ(street.counts(), 0); }
      }
    }
  }
}

TEST_CASE("Road") {
  SUBCASE("Constructor") {
    GIVEN("Valid road parameters") {
      WHEN("A road is constructed with basic parameters") {
        Street road{1, std::make_pair(0, 1), 100.0, 13.8888888889, 2, "Main Street"};
        THEN("All parameters are set correctly") {
          CHECK_EQ(road.id(), 1);
          CHECK_EQ(road.nodePair().first, 0);
          CHECK_EQ(road.nodePair().second, 1);
          CHECK_EQ(road.length(), 100.0);
          CHECK_EQ(road.maxSpeed(), 13.8888888889);
          CHECK_EQ(road.nLanes(), 2);
          CHECK_EQ(road.name(), "Main Street");
          // capacity = ceil((length * nLanes) / meanVehicleLength)
          CHECK_EQ(road.capacity(),
                   static_cast<int>(std::ceil((100.0 * 2) / Road::meanVehicleLength())));
          CHECK_EQ(road.transportCapacity(), 1.0);
          CHECK_EQ(road.priority(), 200);  // 2 * 100
        }
      }

      WHEN("A road is constructed with explicit capacity") {
        Street road{2,
                    std::make_pair(1, 2),
                    50.0,
                    20.0,
                    1,
                    "Side Street",
                    dsf::geometry::PolyLine{},
                    25,
                    2.5};
        THEN("Capacity is set to explicit value") {
          CHECK(road.geometry().empty());
          CHECK_EQ(road.capacity(), 25);
          CHECK_EQ(road.transportCapacity(), 2.5);
        }
      }
    }

    GIVEN("Invalid road parameters") {
      WHEN("Length is non-positive") {
        THEN("Constructor throws invalid_argument") {
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), 0.0), std::invalid_argument);
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), -5.0), std::invalid_argument);
        }
      }

      WHEN("Max speed is non-positive") {
        THEN("Constructor throws invalid_argument") {
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), 100.0, 0.0),
                          std::invalid_argument);
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), 100.0, -10.0),
                          std::invalid_argument);
        }
      }

      WHEN("Number of lanes is zero or negative") {
        THEN("Constructor throws invalid_argument") {
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), 100.0, 13.8888888889, 0),
                          std::invalid_argument);
          CHECK_THROWS_AS(Street(1, std::make_pair(0, 1), 100.0, 13.8888888889, -1),
                          std::invalid_argument);
        }
      }

      WHEN("Transport capacity is non-positive") {
        THEN("Constructor throws invalid_argument") {
          CHECK_THROWS_AS(Street(1,
                                 std::make_pair(0, 1),
                                 100.0,
                                 13.8888888889,
                                 1,
                                 "",
                                 dsf::geometry::PolyLine{},
                                 std::nullopt,
                                 0.0),
                          std::invalid_argument);
          CHECK_THROWS_AS(Street(1,
                                 std::make_pair(0, 1),
                                 100.0,
                                 13.8888888889,
                                 1,
                                 "",
                                 dsf::geometry::PolyLine{},
                                 std::nullopt,
                                 -1.0),
                          std::invalid_argument);
        }
      }
    }
  }

  SUBCASE("Mean Vehicle Length") {
    GIVEN("Default mean vehicle length") {
      THEN("It should be positive") { CHECK(Road::meanVehicleLength() > 0.0); }
    }

    WHEN("Mean vehicle length is set to a valid value") {
      Road::setMeanVehicleLength(4.5);
      THEN("The new value is returned") { CHECK_EQ(Road::meanVehicleLength(), 4.5); }
    }

    WHEN("Mean vehicle length is set to invalid value") {
      THEN("setMeanVehicleLength throws invalid_argument") {
        CHECK_THROWS_AS(Road::setMeanVehicleLength(0.0), std::invalid_argument);
        CHECK_THROWS_AS(Road::setMeanVehicleLength(-1.0), std::invalid_argument);
      }
    }
  }

  SUBCASE("Setters") {
    Street road{1, std::make_pair(0, 1), 100.0};

    SUBCASE("setMaxSpeed") {
      WHEN("Valid speed is set") {
        road.setMaxSpeed(25.0);
        THEN("Max speed is updated") { CHECK_EQ(road.maxSpeed(), 25.0); }
      }

      WHEN("Invalid speed is set") {
        THEN("setMaxSpeed throws invalid_argument") {
          CHECK_THROWS_AS(road.setMaxSpeed(0.0), std::invalid_argument);
          CHECK_THROWS_AS(road.setMaxSpeed(-5.0), std::invalid_argument);
        }
      }
    }

    SUBCASE("setCapacity") {
      WHEN("Valid capacity is set") {
        road.setCapacity(50);
        THEN("Capacity is updated") { CHECK_EQ(road.capacity(), 50); }
      }

      WHEN("Invalid capacity is set") {
        THEN("setCapacity throws invalid_argument") {
          CHECK_THROWS_AS(road.setCapacity(0), std::invalid_argument);
          CHECK_THROWS_AS(road.setCapacity(-1), std::invalid_argument);
        }
      }
    }

    SUBCASE("setTransportCapacity") {
      WHEN("Valid transport capacity is set") {
        road.setTransportCapacity(3.5);
        THEN("Transport capacity is updated") { CHECK_EQ(road.transportCapacity(), 3.5); }
      }

      WHEN("Invalid transport capacity is set") {
        THEN("setTransportCapacity throws invalid_argument") {
          CHECK_THROWS_AS(road.setTransportCapacity(0.0), std::invalid_argument);
          CHECK_THROWS_AS(road.setTransportCapacity(-2.0), std::invalid_argument);
        }
      }
    }

    SUBCASE("setPriority") {
      WHEN("Priority is set") {
        road.setPriority(150);
        THEN("Priority is updated") { CHECK_EQ(road.priority(), 150); }
      }
    }
  }

  SUBCASE("Forbidden Turns") {
    Street road{1, std::make_pair(0, 1), 100.0};

    SUBCASE("addForbiddenTurn") {
      WHEN("A forbidden turn is added") {
        road.addForbiddenTurn(5);
        THEN("The forbidden turn is in the set") {
          CHECK(road.forbiddenTurns().count(5) == 1);
        }

        WHEN("Another forbidden turn is added") {
          road.addForbiddenTurn(10);
          THEN("Both forbidden turns are in the set") {
            CHECK(road.forbiddenTurns().count(5) == 1);
            CHECK(road.forbiddenTurns().count(10) == 1);
          }
        }
      }
    }

    SUBCASE("setForbiddenTurns") {
      WHEN("Forbidden turns are set") {
        std::set<dsf::Id> turns{2, 4, 6};
        road.setForbiddenTurns(turns);
        THEN("The forbidden turns are set correctly") {
          CHECK_EQ(road.forbiddenTurns(), turns);
        }
      }
    }
  }

  SUBCASE("Getters") {
    Street road{1, std::make_pair(0, 1), 100.0, 20.0, 2, "Test Road"};

    CHECK_EQ(road.length(), 100.0);
    CHECK_EQ(road.maxSpeed(), 20.0);
    CHECK_EQ(road.nLanes(), 2);
    CHECK_EQ(road.capacity(),
             static_cast<int>(std::ceil((100.0 * 2) / Road::meanVehicleLength())));
    CHECK_EQ(road.transportCapacity(), 1.0);
    CHECK_EQ(road.name(), "Test Road");
    CHECK_EQ(road.priority(), 200);  // 2 * 100
    CHECK(road.forbiddenTurns().empty());
  }

  SUBCASE("turnDirection") {
    Street road{1, std::make_pair(0, 1), 100.0};
    road.setGeometry(
        dsf::geometry::PolyLine{dsf::geometry::Point{0, 0}, dsf::geometry::Point{1, 0}});

    WHEN("Previous street angle is straight ahead") {
      double previousAngle = 0.0;
      THEN("Turn direction is STRAIGHT") {
        CHECK_EQ(road.turnDirection(previousAngle), dsf::Direction::STRAIGHT);
      }
    }

    WHEN("Previous street angle creates small right turn") {
      double previousAngle = -std::numbers::pi / 16;  // Small negative angle
      THEN("Turn direction is STRAIGHT") {
        CHECK_EQ(road.turnDirection(previousAngle), dsf::Direction::STRAIGHT);
      }
    }

    WHEN("Previous street angle creates right turn") {
      double previousAngle = std::numbers::pi / 4;  // 45 degrees
      THEN("Turn direction is RIGHT") {
        CHECK_EQ(road.turnDirection(previousAngle), dsf::Direction::RIGHT);
      }
    }

    WHEN("Previous street angle creates left turn") {
      double previousAngle = -std::numbers::pi / 4;  // -45 degrees
      THEN("Turn direction is LEFT") {
        CHECK_EQ(road.turnDirection(previousAngle), dsf::Direction::LEFT);
      }
    }

    WHEN("Previous street angle creates U-turn") {
      double previousAngle = std::numbers::pi;  // exactly pi
      THEN("Turn direction is UTURN") {
        CHECK_EQ(road.turnDirection(previousAngle), dsf::Direction::UTURN);
      }

      double previousAngle2 = -std::numbers::pi;  // -pi
      THEN("Turn direction is UTURN") {
        CHECK_EQ(road.turnDirection(previousAngle2), dsf::Direction::UTURN);
      }
    }
  }
}

TEST_CASE("Street formatting") {
  Road::setMeanVehicleLength(1.);

  SUBCASE("std::format with unnamed street") {
    Street street{10, std::make_pair(5, 8), 100.0, 13.89, 2};

    std::string formatted = std::format("{}", street);
    CHECK(formatted.find("Street(id: 10") != std::string::npos);
    CHECK(formatted.find("from 5 to 8") != std::string::npos);
    CHECK(formatted.find("length: 100") != std::string::npos);
    CHECK(formatted.find("max speed: 13.89 m/s") != std::string::npos);
    CHECK(formatted.find("lanes: 2") != std::string::npos);
    CHECK(formatted.find("agents: 0") != std::string::npos);
    CHECK(formatted.find("n enqueued: 0") != std::string::npos);
  }

  SUBCASE("std::format with named street") {
    Street street{20, std::make_pair(1, 2), 250.5, 25.0, 3, "Main Street"};

    std::string formatted = std::format("{}", street);
    CHECK(formatted.find("Street(id: 20 \"Main Street\"") != std::string::npos);
    CHECK(formatted.find("from 1 to 2") != std::string::npos);
    CHECK(formatted.find("length: 250.5") != std::string::npos);
    CHECK(formatted.find("max speed: 25.00 m/s") != std::string::npos);
    CHECK(formatted.find("lanes: 3") != std::string::npos);
  }

  SUBCASE("std::format with street containing agents") {
    Street street{30, std::make_pair(10, 20), 50.0, 15.0, 1};

    // Add some agents
    auto agent1 = std::make_unique<Agent>(0, 1);
    auto agent2 = std::make_unique<Agent>(0, 2);
    street.addAgent(std::move(agent1));
    street.addAgent(std::move(agent2));

    std::string formatted = std::format("{}", street);
    CHECK(formatted.find("Street(id: 30") != std::string::npos);
    CHECK(formatted.find("agents: 2") != std::string::npos);
  }
}
