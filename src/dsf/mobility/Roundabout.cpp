#include "Roundabout.hpp"

#include <format>
#include <stdexcept>

namespace dsf::mobility {
  Roundabout::Roundabout(const RoadJunction& node) : RoadJunction{node} {}

  void Roundabout::enqueue(std::unique_ptr<Agent> pAgent) {
    assert(!isFull());
    m_agents.push(std::move(pAgent));
  }

  std::unique_ptr<Agent> Roundabout::dequeue() {
    assert(!m_agents.empty());
    return m_agents.extract_front();
  }
}  // namespace dsf::mobility