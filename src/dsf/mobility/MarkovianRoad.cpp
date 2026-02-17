#include "MarkovianRoad.hpp"

namespace dsf::mobility {
  void MarkovianRoad::enqueue(std::unique_ptr<Agent> pAgent) {
    if (this->isFull()) {
      throw std::overflow_error("Cannot add agent to a full road.");
    }
    m_queue.push(std::move(pAgent));
  }

  std::unique_ptr<Agent> MarkovianRoad::dequeue() {
    if (m_queue.empty()) {
      throw std::underflow_error("Cannot remove agent from an empty road.");
    }
    auto pAgent{std::move(m_queue.front())};
    m_queue.pop();
    return pAgent;
  }
}  // namespace dsf::mobility