#include "../headers/TrafficLight.hpp"

#include <format>
#include <numeric>
#include <stdexcept>

namespace dsf {
  bool TrafficLightCycle::isGreen(Delay const cycleTime, Delay const counter) const {
    auto const greenStart = m_values.second % cycleTime;
    auto const greenEnd = (m_values.second + m_values.first) % cycleTime;

    if (greenStart < greenEnd) {
      // Normal case: green does not wrap around
      return (counter >= greenStart) && (counter < greenEnd);
    } else {
      // Wraparound case: green spans cycle boundary
      return (counter >= greenStart) || (counter < greenEnd);
    }
  }
  void TrafficLightCycle::reset() { m_values = m_defaultValues; }

  bool TrafficLight::m_allowFreeTurns{true};
  void TrafficLight::setAllowFreeTurns(bool allow) { m_allowFreeTurns = allow; }

  void TrafficLight::moveCycle(Id const oldStreetId, Id const newStreetId) {
    std::vector<TrafficLightCycle> newCycles;
    for (auto& cycle : m_cycles) {
      std::set<std::pair<Id, Direction>> associations;
      for (auto const& [streetId, dir] : cycle.associations()) {
        if (streetId == oldStreetId) {
          associations.emplace(newStreetId, dir);
        } else {
          associations.emplace(streetId, dir);
        }
      }
      newCycles.push_back(
          TrafficLightCycle{associations, cycle.greenTime(), cycle.phase()});
    }
    m_cycles = std::move(newCycles);
  }

  TrafficLight& TrafficLight::operator++() {
    m_counter = (m_counter + 1) % m_cycleTime;
    return *this;
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    auto pair = std::make_pair(streetId, direction);
    for (auto const& cycle : m_cycles) {
      if (cycle.associations().contains(pair)) {
        return cycle.isGreen(m_cycleTime, m_counter);
      }
    }
    throw std::invalid_argument(Logger::buildExceptionMessage(
        std::format("Street id {} with direction {} is not valid for node {}.",
                    streetId,
                    directionToString.at(direction),
                    id())));
  }

  void TrafficLight::resetCycles() {
    m_defaultCycles.empty() ? m_defaultCycles = m_cycles : m_cycles = m_defaultCycles;
  }
}  // namespace dsf