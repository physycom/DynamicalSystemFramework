#include "Intersection.hpp"

#include <stdexcept>

namespace dsf::mobility {
  void Intersection::setCapacity(Size capacity) {
    if (capacity < m_agents.size()) {
      throw std::runtime_error(std::format(
          "Intersection capacity ({}) is smaller than the current queue size ({}).",
          capacity,
          m_agents.size()));
    }
    RoadJunction::setCapacity(capacity);
  }

  void Intersection::addAgent(double angle, std::unique_ptr<Agent> pAgent) {
    if (isFull()) {
      throw std::runtime_error(std::format("{} is full.", *this));
    }
    auto iAngle{static_cast<int16_t>(angle * 100)};
    m_agents.emplace(iAngle, std::move(pAgent));
  }

  void Intersection::addAgent(std::unique_ptr<Agent> pAgent) {
    int lastKey{0};
    if (!m_agents.empty()) {
      lastKey = m_agents.rbegin()->first + 1;
    }
    addAgent(static_cast<double>(lastKey), std::move(pAgent));
  }
}  // namespace dsf::mobility