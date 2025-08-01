#include "../headers/Edge.hpp"
#include "../headers/../utility/Logger.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsf {
  Edge::Edge(Id id,
             std::pair<Id, Id> nodePair,
             int capacity,
             double transportCapacity,
             std::vector<std::pair<double, double>> geometry)
      : m_geometry{std::move(geometry)},
        m_id(id),
        m_nodePair(nodePair),
        m_capacity{capacity},
        m_transportCapacity{transportCapacity} {
    if (capacity < 1) {
      Logger::error(std::format("Edge capacity ({}) must be greater than 0.", capacity));
    }
    assert(transportCapacity > 0.);
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }

  void Edge::m_setAngle(std::pair<double, double> srcNodeCoordinates,
                        std::pair<double, double> dstNodeCoordinates) {
    // N.B.: lat, lon <==> y, x
    double const dx{dstNodeCoordinates.first - srcNodeCoordinates.first};
    double const dy{dstNodeCoordinates.second - srcNodeCoordinates.second};
    m_angle = std::atan2(dy, dx);
    if (m_angle < 0.) {
      m_angle += 2 * std::numbers::pi;
    }
    assert(!(std::abs(m_angle) > 2 * std::numbers::pi));
  }

  void Edge::resetId(Id newId) { m_id = newId; }
  void Edge::setCapacity(int capacity) {
    if (capacity < 1) {
      Logger::error(std::format("Edge capacity ({}) must be greater than 0.", capacity));
    }
    m_capacity = capacity;
  }
  void Edge::setTransportCapacity(double capacity) {
    assert(capacity > 0.);
    m_transportCapacity = capacity;
  }

  void Edge::setGeometry(std::vector<std::pair<double, double>> geometry) {
    m_geometry = std::move(geometry);
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }

  Id Edge::id() const { return m_id; }
  Id Edge::source() const { return m_nodePair.first; }
  Id Edge::target() const { return m_nodePair.second; }
  std::pair<Id, Id> const& Edge::nodePair() const { return m_nodePair; }
  int Edge::capacity() const { return m_capacity; }
  double Edge::transportCapacity() const { return m_transportCapacity; }
  double Edge::angle() const { return m_angle; }
  std::vector<std::pair<double, double>> const& Edge::geometry() const {
    return m_geometry;
  }

  double Edge::deltaAngle(double const previousEdgeAngle) const {
    double deltaAngle{m_angle - previousEdgeAngle};
    if (deltaAngle > std::numbers::pi) {
      deltaAngle -= 2 * std::numbers::pi;
    } else if (deltaAngle < -std::numbers::pi) {
      deltaAngle += 2 * std::numbers::pi;
    }
    return deltaAngle;
  }
};  // namespace dsf