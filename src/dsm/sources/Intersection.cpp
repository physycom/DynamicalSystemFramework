#include "../headers/Intersection.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace dsm {
  void Intersection::setCapacity(Size capacity) {
    if (capacity < m_agents.size()) {
      throw std::runtime_error(Logger::buildExceptionMessage(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          m_agents.size())));
    }
    Node::setCapacity(capacity);
  }

  void Intersection::addAgent(double angle, std::unique_ptr<Agent> pAgent) {
    assert(!isFull());
    auto iAngle{static_cast<int16_t>(angle * 100)};
    m_agents.emplace(iAngle, (std::move(pAgent)));
    ++m_agentCounter;
  }

  void Intersection::addAgent(std::unique_ptr<Agent> pAgent) {
    int lastKey{0};
    if (!m_agents.empty()) {
      lastKey = m_agents.rbegin()->first + 1;
    }
    addAgent(static_cast<double>(lastKey), std::move(pAgent));
  }

  // std::unique_ptr<Agent> Intersection::removeAgent() {
  //   std::erase_if(m_agents,
  //                 [agentId](const auto& pair) { return pair.second == agentId; });
  // }

  Size Intersection::agentCounter() {
    Size copy{m_agentCounter};
    m_agentCounter = 0;
    return copy;
  }
}  // namespace dsm