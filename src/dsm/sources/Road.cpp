#include "../headers/Road.hpp"
#include "../headers/../utility/Logger.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <stdexcept>

namespace dsm {
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
      Logger::error(std::format("The road length ({}) must be greater than 0.", length));
    }
    if (!(maxSpeed > 0.)) {
      Logger::error(std::format(
          "The maximum speed of a road ({}) must be greater than 0.", maxSpeed));
    }
    if (nLanes < 1) {
      Logger::error(std::format(
          "The number of lanes of a road ({}) must be greater than 0.", nLanes));
    }
  }
  void Road::setMeanVehicleLength(double meanVehicleLength) {
    if (!(meanVehicleLength > 0.)) {
      Logger::error(std::format("The mean vehicle length ({}) must be greater than 0.",
                                meanVehicleLength));
    }
    m_meanVehicleLength = meanVehicleLength;
  }
  double Road::meanVehicleLength() { return m_meanVehicleLength; }

  void Road::addForbiddenTurn(Id roadId) { m_forbiddenTurns.insert(roadId); }
  void Road::setForbiddenTurns(std::set<Id> const& forbiddenTurns) {
    m_forbiddenTurns = forbiddenTurns;
  }

  void Road::setMaxSpeed(double speed) {
    if (speed < 0.) {
      Logger::error(
          std::format("The maximum speed of a road ({}) cannot be negative.", speed));
    }
    m_maxSpeed = speed;
  }
  void Road::setPriority(int priority) {
    assert(priority >= 0);
    m_priority = priority;
  }

  double Road::length() const { return m_length; }
  double Road::maxSpeed() const { return m_maxSpeed; }
  int Road::nLanes() const { return m_nLanes; }
  std::string Road::name() const { return m_name; }
  int Road::priority() const { return m_priority; }
  std::set<Id> const& Road::forbiddenTurns() const { return m_forbiddenTurns; }
};  // namespace dsm