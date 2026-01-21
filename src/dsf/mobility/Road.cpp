#include "Road.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  double Road::m_meanVehicleLength = 5.;

  Road::Road(Id id,
             std::pair<Id, Id> nodePair,
             double length,
             double maxSpeed,
             int nLanes,
             std::string name,
             geometry::PolyLine polyline,
             std::optional<int> capacity,
             double transportCapacity)
      : Edge(id, std::move(nodePair), std::move(polyline)),
        m_length{length},
        m_maxSpeed{maxSpeed},
        m_nLanes{nLanes},
        m_name{std::move(name)} {
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
    m_capacity = capacity.value_or(std::ceil((length * nLanes) / m_meanVehicleLength));
    if (m_capacity < 1) {
      throw std::invalid_argument(
          std::format("The capacity of a road ({}) must be greater than 0.", m_capacity));
    }
    if (transportCapacity <= 0.) {
      throw std::invalid_argument(
          std::format("The transport capacity of a road ({}) must be greater than 0.",
                      transportCapacity));
    }
    m_transportCapacity = transportCapacity;
    switch (nLanes) {
      case 1:
        m_laneMapping.emplace_back(Direction::ANY);
        break;
      case 2:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      case 3:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::STRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      default:
        m_laneMapping.emplace_back(Direction::RIGHT);
        for (auto i{1}; i < nLanes - 1; ++i) {
          m_laneMapping.emplace_back(Direction::STRAIGHT);
        }
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
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
  void Road::setCapacity(int capacity) {
    if (capacity < 1) {
      throw std::invalid_argument(
          std::format("The capacity of a road ({}) must be greater than 0.", capacity));
    }
    m_capacity = capacity;
  }
  void Road::setTransportCapacity(double transportCapacity) {
    if (transportCapacity <= 0.) {
      throw std::invalid_argument(
          std::format("The transport capacity of a road ({}) must be greater than 0.",
                      transportCapacity));
    }
    m_transportCapacity = transportCapacity;
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
  void Road::setLaneMapping(std::vector<Direction> const& laneMapping) {
    assert(laneMapping.size() == static_cast<size_t>(m_nLanes));
    m_laneMapping = laneMapping;
    std::string strLaneMapping;
    std::for_each(
        laneMapping.cbegin(), laneMapping.cend(), [&strLaneMapping](auto const item) {
          strLaneMapping +=
              std::format("{} - ", directionToString[static_cast<size_t>(item)]);
        });
    spdlog::debug("New lane mapping for road {} -> {} is: {}",
                  m_nodePair.first,
                  m_nodePair.second,
                  strLaneMapping);
  }
};  // namespace dsf::mobility