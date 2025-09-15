#include "../headers/Road.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsf {
  double Road::m_meanVehicleLength = 5.;

  Road::Road(Id id,
             std::pair<Id, Id> nodePair,
             double length,
             double maxSpeed,
             int nLanes,
             std::string name,
             std::vector<std::pair<double, double>> geometry,
             std::optional<int> capacity,
             double transportCapacity)
      : Edge(id,
             std::move(nodePair),
             capacity.value_or(std::ceil((length * nLanes) / m_meanVehicleLength)),
             transportCapacity,
             std::move(geometry)),
        m_length{length},
        m_maxSpeed{maxSpeed},
        m_nLanes{nLanes},
        m_name{std::move(name)},
        m_priority{nLanes * 100} {
    if (!(length > 0.)) {
      throw std::invalid_argument(
          std::format("The length of a road ({}) must be greater than 0.", length));
    }
    if (!(maxSpeed > 0.)) {
      throw std::invalid_argument(std::format(
          "The maximum speed of a road ({}) must be greater than 0.", maxSpeed));
    }
    if (nLanes < 1) {
      throw std::invalid_argument(std::format(
          "The number of lanes of a road ({}) must be greater than 0.", nLanes));
    }
  }
  void Road::setMeanVehicleLength(double meanVehicleLength) {
    if (!(meanVehicleLength > 0.)) {
      throw std::invalid_argument(std::format(
          "The mean vehicle length ({}) must be greater than 0.", meanVehicleLength));
    }
    m_meanVehicleLength = meanVehicleLength;
  }
  double Road::meanVehicleLength() { return m_meanVehicleLength; }

  void Road::addForbiddenTurn(Id roadId) { m_forbiddenTurns.insert(roadId); }
  void Road::setForbiddenTurns(std::set<Id> const& forbiddenTurns) {
    m_forbiddenTurns = forbiddenTurns;
  }

  void Road::setMaxSpeed(double speed) {
    if (speed <= 0.) {
      throw std::invalid_argument(
          std::format("The maximum speed of a road ({}) must be greater than 0.", speed));
    }
    m_maxSpeed = speed;
  }
  void Road::setPriority(int priority) {
    assert(priority >= 0);
    m_priority = priority;
  }
  Direction Road::turnDirection(double const& previousStreetAngle) const {
    auto const deltaAngle{this->deltaAngle(previousStreetAngle)};
    if (std::abs(deltaAngle) >= std::numbers::pi) {
      return Direction::UTURN;
    }
    if (std::abs(deltaAngle) < std::numbers::pi / 8) {
      return Direction::STRAIGHT;
    }
    if (deltaAngle < 0.) {
      return Direction::RIGHT;
    }
    return Direction::LEFT;
  }

  double Road::length() const { return m_length; }
  double Road::maxSpeed() const { return m_maxSpeed; }
  int Road::nLanes() const { return m_nLanes; }
  std::string Road::name() const { return m_name; }
  int Road::priority() const { return m_priority; }
  std::set<Id> const& Road::forbiddenTurns() const { return m_forbiddenTurns; }
};  // namespace dsf