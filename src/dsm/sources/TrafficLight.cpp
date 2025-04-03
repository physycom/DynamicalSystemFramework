#include "../headers/TrafficLight.hpp"

#include <format>
#include <numeric>
#include <stdexcept>

namespace dsm {
  bool TrafficLightCycle::isGreen(Delay const cycleTime, Delay const counter) const {
    auto const rest{(m_phase + m_greenTime) / cycleTime};
    if (rest) {
      return (counter < rest) || (counter >= m_phase);
    }
    return (counter >= m_phase) && (counter < m_phase + m_greenTime);
  }

  bool TrafficLight::m_allowFreeTurns{true};
  void TrafficLight::setAllowFreeTurns(bool allow) { m_allowFreeTurns = allow; }
  void TrafficLight::setCycle(Id const streetId,
                              Direction direction,
                              TrafficLightCycle const& cycle) {
    if ((cycle.greenTime() > m_cycleTime)) {
      Logger::error(std::format("Green time ({}) must not exceed the cycle time ({}).",
                                cycle.greenTime(),
                                m_cycleTime));
    }
    if (!(cycle.phase() < m_cycleTime)) {
      Logger::error(std::format("Phase ({}) must be less than the cycle time ({}).",
                                cycle.phase(),
                                m_cycleTime));
    }
    m_cycles[streetId].emplace(direction, cycle);
  }

  void TrafficLightCycle::reset() {
    m_greenTime = m_defaultValues.first;
    m_phase = m_defaultValues.second;
  }

  void TrafficLight::setComplementaryCycle(Id const streetId, Id const existingCycle) {
    if (m_cycles.contains(streetId)) {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("Street id already exists."));
    }
    if (!m_cycles.contains(existingCycle)) {
      throw std::invalid_argument(Logger::buildExceptionMessage("Cycle does not exist."));
    }
    m_cycles.emplace(streetId, m_cycles.at(existingCycle));
    for (auto& [direction, cycle] : m_cycles.at(streetId)) {
      cycle = TrafficLightCycle(m_cycleTime - cycle.greenTime(),
                                cycle.phase() + m_cycleTime - cycle.greenTime());
    }
  }

  void TrafficLight::moveCycle(Id const oldStreetId, Id const newStreetId) {
    if (!m_cycles.contains(oldStreetId)) {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("Old street id does not exist."));
    }
    auto handler{m_cycles.extract(oldStreetId)};
    handler.key() = newStreetId;
    m_cycles.insert(std::move(handler));
  }

  TrafficLight& TrafficLight::operator++() {
    m_counter = (m_counter + 1) % m_cycleTime;
    return *this;
  }

  Delay TrafficLight::maxGreenTime(bool priorityStreets) const {
    Delay maxTime{0};
    for (auto const& [streetId, cycles] : m_cycles) {
      if (priorityStreets && m_streetPriorities.contains(streetId)) {
        for (auto const& [direction, cycle] : cycles) {
          maxTime = std::max(maxTime, cycle.greenTime());
        }
      } else {
        for (auto const& [direction, cycle] : cycles) {
          maxTime = std::max(maxTime, cycle.greenTime());
        }
      }
    }
    return maxTime;
  }

  Delay TrafficLight::minGreenTime(bool priorityStreets) const {
    Delay minTime{std::numeric_limits<Delay>::max()};
    for (auto const& [streetId, cycles] : m_cycles) {
      if (priorityStreets && m_streetPriorities.contains(streetId)) {
        for (auto const& [direction, cycle] : cycles) {
          minTime = std::min(minTime, cycle.greenTime());
        }
      } else {
        for (auto const& [direction, cycle] : cycles) {
          minTime = std::min(minTime, cycle.greenTime());
        }
      }
    }
    return minTime;
  }

  double TrafficLight::meanGreenTime(bool priorityStreets) const {
    double meanTime{0.};
    size_t nCycles{0};
    for (auto const& [streetId, cycles] : m_cycles) {
      if ((priorityStreets && m_streetPriorities.contains(streetId)) ||
          (!priorityStreets && !m_streetPriorities.contains(streetId))) {
        meanTime +=
            std::transform_reduce(cycles.begin(),
                                  cycles.end(),
                                  0.0,                  // Initial value (double)
                                  std::plus<double>(),  // Reduction function (addition)
                                  [](const auto& pair) -> double {
                                    return static_cast<double>(pair.second.greenTime());
                                  });
        nCycles += cycles.size();
      }
    }
    return meanTime / nCycles;
  }

  void TrafficLight::increaseGreenTimes(Delay const delta) {
    for (auto& [streetId, cycles] : m_cycles) {
      if (m_streetPriorities.contains(streetId)) {
        for (auto& [direction, cycle] : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() + delta, cycle.phase());
        }
      } else {
        for (auto& [direction, cycle] : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() - delta, cycle.phase() + delta);
        }
      }
    }
  }

  void TrafficLight::decreaseGreenTimes(Delay const delta) {
    for (auto& [streetId, cycles] : m_cycles) {
      if (!m_streetPriorities.contains(streetId)) {
        for (auto& [direction, cycle] : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() + delta, cycle.phase());
        }
      } else {
        for (auto& [direction, cycle] : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() - delta, cycle.phase() + delta);
        }
      }
    }
  }

  bool TrafficLight::isDefault() const {
    for (auto const& [streetId, cycles] : m_cycles) {
      for (auto const& [direction, cycle] : cycles) {
        if (!cycle.isDefault()) {
          return false;
        }
      }
    }
    return true;
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    if (!m_cycles.contains(streetId)) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street id {} is not valid for node {}.", streetId, id())));
    }
    if (!m_cycles.at(streetId).contains(direction)) {
      switch (direction) {
        case Direction::RIGHT:
          if (m_cycles.at(streetId).contains(Direction::RIGHTANDSTRAIGHT)) {
            direction = Direction::RIGHTANDSTRAIGHT;
          } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
            direction = Direction::ANY;
          }
          break;
        case Direction::LEFT:
          if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
            direction = Direction::LEFTANDSTRAIGHT;
          } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
            direction = Direction::ANY;
          }
          break;
        case Direction::STRAIGHT:
          if (m_cycles.at(streetId).contains(Direction::RIGHTANDSTRAIGHT)) {
            direction = Direction::RIGHTANDSTRAIGHT;
          } else if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
            direction = Direction::LEFTANDSTRAIGHT;
          } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
            direction = Direction::ANY;
          }
          break;
        case Direction::UTURN:
          if (m_cycles.at(streetId).contains(Direction::LEFT)) {
            direction = Direction::LEFT;
          } else if (m_cycles.at(streetId).contains(Direction::LEFTANDSTRAIGHT)) {
            direction = Direction::LEFTANDSTRAIGHT;
          } else if (m_cycles.at(streetId).contains(Direction::ANY)) {
            direction = Direction::ANY;
          }
          break;
        default:
          if (m_cycles.at(streetId).contains(Direction::ANY)) {
            direction = Direction::ANY;
          } else {
            Logger::warning(std::format(
                "Street {} has ...ANDSTRAIGHT phase but Traffic Light {} doesn't.",
                streetId,
                m_id));
          }
      }
    }
    if (!m_cycles.at(streetId).contains(direction)) {
      return m_allowFreeTurns;
    }
    return m_cycles.at(streetId).at(direction).isGreen(m_cycleTime, m_counter);
  }

  bool TrafficLight::isFavouringDirection(bool const priority) const {
    for (auto const& [streetId, cycles] : m_cycles) {
      if ((priority && m_streetPriorities.contains(streetId)) ||
          (!priority && !m_streetPriorities.contains(streetId))) {
        for (auto const& [direction, cycle] : cycles) {
          if (!cycle.isGreenTimeIncreased()) {
            return false;
          }
        }
      }
    }
    return true;
  }

  void TrafficLight::resetCycles() {
    for (auto& [streetId, cycles] : m_cycles) {
      for (auto& [direction, cycle] : cycles) {
        cycle.reset();
      }
    }
  }
}  // namespace dsm