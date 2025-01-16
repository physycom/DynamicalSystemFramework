#include "Edge.hpp"
#include "../utility/Logger.hpp"

#include <stdexcept>
#include <format>

namespace dsm {
  Edge::Edge(Id id, std::pair<Id, Id> nodePair, int capacity, int transportCapacity)
      : m_id(id),
        m_nodePair(nodePair),
        m_capacity{capacity},
        m_transportCapacity{transportCapacity} {
    if (capacity < 1) {
      throw std::invalid_argument(
          buildLog(std::format("Edge capacity ({}) must be greater than 0.", capacity)));
    }
    if (transportCapacity < 1) {
      throw std::invalid_argument(buildLog(std::format(
          "Edge transport capacity ({}) must be greater than 0.", transportCapacity)));
    }
  }

  void Edge::setCapacity(int capacity) {
    if (capacity < 1) {
      throw std::invalid_argument(
          buildLog(std::format("Edge capacity ({}) must be greater than 0.", capacity)));
    }
    m_capacity = capacity;
  }
  void Edge::setTransportCapacity(int capacity) {
    if (capacity < 1) {
      throw std::invalid_argument(buildLog(
          std::format("Edge transport capacity ({}) must be greater than 0.", capacity)));
    }
    m_transportCapacity = capacity;
  }

  Id Edge::id() const { return m_id; }
  Id Edge::u() const { return m_nodePair.first; }
  Id Edge::v() const { return m_nodePair.second; }
  std::pair<Id, Id> Edge::nodePair() const { return m_nodePair; }
  int Edge::capacity() const { return m_capacity; }
  int Edge::transportCapacity() const { return m_transportCapacity; }
};  // namespace dsm