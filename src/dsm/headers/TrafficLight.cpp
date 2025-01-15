#include "TrafficLight.hpp"

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
      throw std::invalid_argument(buildLog("Green time must not exceed the cycle time."));
    }
    if (!(cycle.phase() < m_cycleTime)) {
      throw std::invalid_argument(buildLog("Phase must be less than the cycle time."));
    }
    if (direction == Direction::UTURN) {
      direction = Direction::LEFT;
    }
    if (!m_cycles.contains(streetId)) {
      TrafficLightCycle defaultCycle(m_cycleTime, 0);
      std::vector<TrafficLightCycle> cycles{defaultCycle, defaultCycle, defaultCycle};
      m_cycles.emplace(streetId, cycles);
    }
    switch (direction) {
      case Direction::RIGHTANDSTRAIGHT:
        m_cycles[streetId][Direction::RIGHT] = cycle;
        m_cycles[streetId][Direction::STRAIGHT] = cycle;
        break;
      case Direction::LEFTANDSTRAIGHT:
        m_cycles[streetId][Direction::LEFT] = cycle;
        m_cycles[streetId][Direction::STRAIGHT] = cycle;
        break;
      case Direction::ANY:
        m_cycles[streetId][Direction::RIGHT] = cycle;
        m_cycles[streetId][Direction::STRAIGHT] = cycle;
        m_cycles[streetId][Direction::LEFT] = cycle;
        break;
      default:
        m_cycles[streetId][direction] = cycle;
        break;
    }
  }

  void TrafficLightCycle::reset() {
    m_greenTime = m_defaultValues.first;
    m_phase = m_defaultValues.second;
  }

  void TrafficLight::setComplementaryCycle(Id const streetId, Id const existingCycle) {
    if (m_cycles.contains(streetId)) {
      throw std::invalid_argument(buildLog("Street id already exists."));
    }
    if (!m_cycles.contains(existingCycle)) {
      throw std::invalid_argument(buildLog("Cycle does not exist."));
    }
    m_cycles.emplace(streetId, m_cycles.at(existingCycle));
    for (auto& cycle : m_cycles.at(streetId)) {
      cycle = TrafficLightCycle(m_cycleTime - cycle.greenTime(),
                                cycle.phase() + m_cycleTime - cycle.greenTime());
    }
  }

  void TrafficLight::moveCycle(Id const oldStreetId, Id const newStreetId) {
    if (!m_cycles.contains(oldStreetId)) {
      throw std::invalid_argument(buildLog("Old street id does not exist."));
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
        for (auto const& cycle : cycles) {
          maxTime = std::max(maxTime, cycle.greenTime());
        }
      } else {
        for (auto const& cycle : cycles) {
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
        for (auto const& cycle : cycles) {
          minTime = std::min(minTime, cycle.greenTime());
        }
      } else {
        for (auto const& cycle : cycles) {
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
        meanTime += std::accumulate(cycles.begin(),
                                    cycles.end(),
                                    0.,
                                    [](double acc, TrafficLightCycle const& cycle) {
                                      return acc + cycle.greenTime();
                                    });
        nCycles += cycles.size();
      }
    }
    return meanTime / nCycles;
  }

  void TrafficLight::increaseGreenTimes(Delay const delta) {
    for (auto& [streetId, cycles] : m_cycles) {
      if (m_streetPriorities.contains(streetId)) {
        for (auto& cycle : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() + delta, cycle.phase());
        }
      } else {
        for (auto& cycle : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() - delta, cycle.phase() + delta);
        }
      }
    }
  }

  void TrafficLight::decreaseGreenTimes(Delay const delta) {
    for (auto& [streetId, cycles] : m_cycles) {
      if (!m_streetPriorities.contains(streetId)) {
        for (auto& cycle : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() + delta, cycle.phase());
        }
      } else {
        for (auto& cycle : cycles) {
          cycle = TrafficLightCycle(cycle.greenTime() - delta, cycle.phase() + delta);
        }
      }
    }
  }

  bool TrafficLight::isDefault() const {
    for (auto const& [streetId, cycles] : m_cycles) {
      for (auto const& cycle : cycles) {
        if (!cycle.isDefault()) {
          return false;
        }
      }
    }
    return true;
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    if (!m_cycles.contains(streetId)) {
      throw std::invalid_argument(buildLog(
          std::format("Street id {} is not valid for node {}.", streetId, id())));
    }
    switch (direction) {
      case Direction::UTURN:
        direction = Direction::LEFT;
        break;
      case Direction::RIGHTANDSTRAIGHT:
        return m_cycles.at(streetId)[Direction::RIGHT].isGreen(m_cycleTime, m_counter) &&
               m_cycles.at(streetId)[Direction::STRAIGHT].isGreen(m_cycleTime, m_counter);
      case Direction::LEFTANDSTRAIGHT:
        return m_cycles.at(streetId)[Direction::LEFT].isGreen(m_cycleTime, m_counter) &&
               m_cycles.at(streetId)[Direction::STRAIGHT].isGreen(m_cycleTime, m_counter);
      case Direction::ANY:
        return m_cycles.at(streetId)[Direction::RIGHT].isGreen(m_cycleTime, m_counter) &&
               m_cycles.at(streetId)[Direction::STRAIGHT].isGreen(m_cycleTime,
                                                                  m_counter) &&
               m_cycles.at(streetId)[Direction::LEFT].isGreen(m_cycleTime, m_counter);
      default:
        break;
    }
    return m_cycles.at(streetId)[direction].isGreen(m_cycleTime, m_counter);
  }

  void TrafficLight::resetCycles() {
    for (auto& [streetId, cycles] : m_cycles) {
      for (auto& cycle : cycles) {
        cycle.reset();
      }
    }
  }
}  // namespace dsm