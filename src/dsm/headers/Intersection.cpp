#include "Intersection.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace dsm {
  void Intersection::setCapacity(Size capacity) {
    if (capacity < m_agents.size()) {
      throw std::runtime_error(logger.buildExceptionMessage(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          m_agents.size())));
    }
    Node::setCapacity(capacity);
  }

  void Intersection::addAgent(double angle, Id agentId) {
    if (isFull()) {
      throw std::runtime_error(logger.buildExceptionMessage("Intersection is full."));
    }
    for (auto const [angle, id] : m_agents) {
      if (id == agentId) {
        throw std::runtime_error(logger.buildExceptionMessage(
            std::format("Agent with id {} is already on the node.", agentId)));
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
}  // namespace dsm