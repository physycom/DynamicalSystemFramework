
#include "Node.hpp"

#include <cassert>
#include <iostream>

namespace dsm {

  void Intersection::setCapacity(Size capacity) {
    if (capacity < m_agents.size()) {
      throw std::runtime_error(buildLog(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          m_agents.size())));
    }
    Node::setCapacity(capacity);
  }

  void Intersection::addAgent(double angle, Id agentId) {
    if (isFull()) {
      throw std::runtime_error(buildLog("Intersection is full."));
    }
    for (auto const [angle, id] : m_agents) {
      if (id == agentId) {
        throw std::runtime_error(
            buildLog(std::format("Agent with id {} is already on the node.", agentId)));
      }
    }
    auto iAngle{static_cast<int16_t>(angle * 100)};
    m_agents.emplace(iAngle, agentId);
    ++m_agentCounter;
  }

  void Intersection::addAgent(Id agentId) {
    int lastKey{0};
    if (!m_agents.empty()) {
      lastKey = m_agents.rbegin()->first + 1;
    }
    addAgent(static_cast<double>(lastKey), agentId);
  }

  void Intersection::removeAgent(Id agentId) {
    std::erase_if(m_agents,
                  [agentId](const auto& pair) { return pair.second == agentId; });
  }

  Size Intersection::agentCounter() {
    Size copy{m_agentCounter};
    m_agentCounter = 0;
    return copy;
  }

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

  Roundabout::Roundabout(const Node& node) : Node{node.id()} {
    if (node.coords().has_value()) {
      this->setCoords(node.coords().value());
    }
    this->setCapacity(node.capacity());
  }

  void Roundabout::enqueue(Id agentId) {
    if (isFull()) {
      throw std::runtime_error(buildLog("Roundabout is full."));
    }
    for (const auto id : m_agents) {
      if (id == agentId) {
        throw std::runtime_error(buildLog(
            std::format("Agent with id {} is already on the roundabout.", agentId)));
      }
    }
    m_agents.push(agentId);
  }

  Id Roundabout::dequeue() {
    if (m_agents.empty()) {
      throw std::runtime_error(buildLog("Roundabout is empty."));
    }
    Id agentId{m_agents.front()};
    m_agents.pop();
    return agentId;
  }

};  // namespace dsm
