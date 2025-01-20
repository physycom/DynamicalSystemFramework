#include "Edge.hpp"
#include "../utility/Logger.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <stdexcept>

namespace dsm {
  Edge::Edge(Id id, std::pair<Id, Id> nodePair, int capacity, int transportCapacity)
      : m_id(id),
        m_nodePair(nodePair),
        m_capacity{capacity},
        m_transportCapacity{transportCapacity},
        m_angle{0.0} {
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

  void Edge::setAngle(std::pair<double, double> srcNodeCoordinates, std::pair<double, double> dstNodeCoordinates) {
    // N.B.: lat, lon <==> y, x
    double const dy{dstNodeCoordinates.first - srcNodeCoordinates.first};
    double const dx{dstNodeCoordinates.second - srcNodeCoordinates.second};
    double angle{std::atan2(dy, dx)};
    if (angle < 0.) {
      angle += 2 * std::numbers::pi;
    }
    assert(!(std::abs(angle) > 2 * std::numbers::pi));
    m_angle = angle;
  }

  Id Edge::id() const { return m_id; }
  Id Edge::u() const { return m_nodePair.first; }
  Id Edge::v() const { return m_nodePair.second; }
  std::pair<Id, Id> Edge::nodePair() const { return m_nodePair; }
  int Edge::capacity() const { return m_capacity; }
  int Edge::transportCapacity() const { return m_transportCapacity; }
  double Edge::angle() const { return m_angle; }
};  // namespace dsm