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
    std::unique_ptr<Agent> pAgent{std::move(m_agents.front())};
    m_agents.pop();
    return pAgent;
  }
}  // namespace dsf::mobility