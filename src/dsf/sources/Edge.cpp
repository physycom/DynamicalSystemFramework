#include "../headers/Edge.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsf {
  Edge::Edge(Id id, std::pair<Id, Id> nodePair, geometry::PolyLine geometry)
      : m_geometry{std::move(geometry)}, m_id(id), m_nodePair(nodePair) {
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }

  void Edge::m_setAngle(geometry::Point srcNodeCoordinates,
                        geometry::Point dstNodeCoordinates) {
    // N.B.: lat, lon <==> y, x
    double const dy{dstNodeCoordinates.y() - srcNodeCoordinates.y()};
    double const dx{dstNodeCoordinates.x() - srcNodeCoordinates.x()};
    m_angle = std::atan2(dy, dx);
    if (m_angle < 0.) {
      m_angle += 2 * std::numbers::pi;
    }
    assert(!(std::abs(m_angle) > 2 * std::numbers::pi));
  }

  void Edge::resetId(Id newId) { m_id = newId; }

  void Edge::setGeometry(geometry::PolyLine geometry) {
    m_geometry = std::move(geometry);
    if (m_geometry.size() > 1) {
      m_setAngle(m_geometry[m_geometry.size() - 2], m_geometry.back());
    } else {
      m_angle = 0.;
    }
  }
  void Edge::setWeight(double const weight) {
    if (weight <= 0.) {
      throw std::invalid_argument(
          std::format("Edge weight ({}) must be greater than 0.", weight));
    }
    m_weight = weight;
  }

  Id Edge::id() const { return m_id; }
  Id Edge::source() const { return m_nodePair.first; }
  Id Edge::target() const { return m_nodePair.second; }
  std::pair<Id, Id> const& Edge::nodePair() const { return m_nodePair; }
  double Edge::angle() const { return m_angle; }
  double Edge::weight() const {
    return m_weight.has_value() ? *m_weight
                                : throw std::runtime_error("Edge weight is not set.");
  }
  geometry::PolyLine const& Edge::geometry() const { return m_geometry; }

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