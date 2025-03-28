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
    if (direction == Direction::UTURN) {
      direction = Direction::LEFT;
    }
    switch (direction) {
      case Direction::RIGHTANDSTRAIGHT:
        m_cycles[streetId].emplace(Direction::RIGHT, cycle);
        m_cycles[streetId].emplace(Direction::STRAIGHT, cycle);
        break;
      case Direction::LEFTANDSTRAIGHT:
        m_cycles[streetId].emplace(Direction::LEFT, cycle);
        m_cycles[streetId].emplace(Direction::STRAIGHT, cycle);
        break;
      case Direction::ANY:
        m_cycles[streetId].emplace(Direction::RIGHT, cycle);
        m_cycles[streetId].emplace(Direction::STRAIGHT, cycle);
        m_cycles[streetId].emplace(Direction::LEFT, cycle);
        break;
      default:
        m_cycles[streetId].emplace(direction, cycle);
        break;
    }
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
    for (auto& pair : m_cycles.at(streetId)) {
      pair.second =
          TrafficLightCycle(m_cycleTime - pair.second.greenTime(),
                            pair.second.phase() + m_cycleTime - pair.second.greenTime());
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
        for (auto const& pair : cycles) {
          maxTime = std::max(maxTime, pair.second.greenTime());
        }
      } else {
        for (auto const& pair : cycles) {
          maxTime = std::max(maxTime, pair.second.greenTime());
        }
      }
    }
    return maxTime;
  }

  Delay TrafficLight::minGreenTime(bool priorityStreets) const {
    Delay minTime{std::numeric_limits<Delay>::max()};
    for (auto const& [streetId, cycles] : m_cycles) {
      if (priorityStreets && m_streetPriorities.contains(streetId)) {
        for (auto const& pair : cycles) {
          minTime = std::min(minTime, pair.second.greenTime());
        }
      } else {
        for (auto const& pair : cycles) {
          minTime = std::min(minTime, pair.second.greenTime());
        }
      }
    }
    return minTime;
  }

  double TrafficLight::meanGreenTime(bool priorityStreets) const {
    double meanTime{0.};
    size_t nCycles{0};
    // for (auto const& [streetId, cycles] : m_cycles) {
    //   if ((priorityStreets && m_streetPriorities.contains(streetId)) ||
    //       (!priorityStreets && !m_streetPriorities.contains(streetId))) {
    //     meanTime +=
    //         std::transform_reduce(cycles.begin(),
    //                               cycles.end(),
    //                               0.0,                  // Initial value (double)
    //                               std::plus<double>(),  // Reduction function (addition)
    //                               [](const TrafficLightCycle& cycle) -> double {
    //                                 return static_cast<double>(cycle.greenTime());
    //                               });
    //     nCycles += cycles.size();
    //   }
    // }
    return meanTime / nCycles;
  }

  void TrafficLight::increaseGreenTimes(Delay const delta) {
    for (auto& [streetId, cycles] : m_cycles) {
      if (m_streetPriorities.contains(streetId)) {
        for (auto& pair : cycles) {
          pair.second =
              TrafficLightCycle(pair.second.greenTime() + delta, pair.second.phase());
        }
      } else {
        for (auto& pair : cycles) {
          pair.second = TrafficLightCycle(pair.second.greenTime() - delta,
                                          pair.second.phase() + delta);
        }
      }
    }
  }

  void TrafficLight::decreaseGreenTimes(Delay const delta) {
    // for (auto& [streetId, cycles] : m_cycles) {
    //   if (!m_streetPriorities.contains(streetId)) {
    //     for (auto& cycle : cycles) {
    //       cycle = TrafficLightCycle(cycle.greenTime() + delta, cycle.phase());
    //     }
    //   } else {
    //     for (auto& cycle : cycles) {
    //       cycle = TrafficLightCycle(cycle.greenTime() - delta, cycle.phase() + delta);
    //     }
    //   }
    // }
  }

  bool TrafficLight::isDefault() const {
    for (auto const& [streetId, cycles] : m_cycles) {
      for (auto const& pair : cycles) {
        if (!pair.second.isDefault()) {
          return false;
        }
      }
    }
    return true;
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    bool bgreen{true};
    // if (m_cycles.empty()) {
    //   return bgreen;
    // }
    if (!m_cycles.contains(streetId)) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street id {} is not valid for node {}.", streetId, id())));
    }
    auto const& cycles{m_cycles.at(streetId)};

    switch (direction) {
      case Direction::UTURN:
        if (cycles.contains(Direction::LEFT)) {
          bgreen == bgreen || cycles.at(Direction::LEFT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        break;
      case Direction::RIGHTANDSTRAIGHT:
        if (cycles.contains(Direction::RIGHT)) {
          bgreen == bgreen || cycles.at(Direction::RIGHT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        if (cycles.contains(Direction::STRAIGHT)) {
          bgreen == bgreen ||
              cycles.at(Direction::STRAIGHT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        break;
      case Direction::LEFTANDSTRAIGHT:
        if (cycles.contains(Direction::STRAIGHT)) {
          bgreen == bgreen ||
              cycles.at(Direction::STRAIGHT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        if (cycles.contains(Direction::LEFT)) {
          bgreen == bgreen || cycles.at(Direction::LEFT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        break;
      case Direction::ANY:
        if (cycles.contains(Direction::RIGHT)) {
          bgreen == bgreen || cycles.at(Direction::RIGHT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        if (cycles.contains(Direction::STRAIGHT)) {
          bgreen == bgreen ||
              cycles.at(Direction::STRAIGHT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        if (cycles.contains(Direction::LEFT)) {
          bgreen == bgreen || cycles.at(Direction::LEFT).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
        break;
      default:
        if (cycles.contains(direction)) {
          bgreen = cycles.at(direction).isGreen(m_cycleTime, m_counter);
        } else {
          bgreen = true;
        }
    }
    return bgreen;
  }

  bool TrafficLight::isFavouringDirection(bool const priority) const {
    for (auto const& [streetId, cycles] : m_cycles) {
      if ((priority && m_streetPriorities.contains(streetId)) ||
          (!priority && !m_streetPriorities.contains(streetId))) {
        for (auto const& pair : cycles) {
          if (!pair.second.isGreenTimeIncreased()) {
            return false;
          }
        }
      }
    }
    return true;
  }

  void TrafficLight::resetCycles() {
    if (m_defaultCycles.empty()) {
      m_defaultCycles = m_cycles;
    } else {
      m_cycles = m_defaultCycles;
    }
  }
}  // namespace dsm