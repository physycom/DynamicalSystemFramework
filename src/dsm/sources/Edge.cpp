#include "../headers/Edge.hpp"
#include "../headers/../utility/Logger.hpp"

#include <cassert>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>

namespace dsm {
  Edge::Edge(Id id,
             std::pair<Id, Id> nodePair,
             int capacity,
             int transportCapacity,
             double angle,
             std::vector<std::pair<double, double>> geometry)
      : m_id(id),
        m_nodePair(nodePair),
        m_capacity{capacity},
        m_transportCapacity{transportCapacity},
        m_angle{angle},
        m_geometry{std::move(geometry)} {
    if (capacity < 1) {
      Logger::error(std::format("Edge capacity ({}) must be greater than 0.", capacity));
    }
    if (transportCapacity < 1) {
      Logger::error(std::format("Edge transport capacity ({}) must be greater than 0.",
                                transportCapacity));
    }
    if (std::abs(angle) > 2 * std::numbers::pi) {
      Logger::error(
          std::format("Edge angle ({}) must be in the range [-2pi, 2pi].", angle));
    }
  }

  void Edge::setCapacity(int capacity) {
    if (capacity < 1) {
      Logger::error(std::format("Edge capacity ({}) must be greater than 0.", capacity));
    }
    m_capacity = capacity;
  }
  void Edge::setTransportCapacity(int capacity) {
    if (capacity < 1) {
      Logger::error(
          std::format("Edge transport capacity ({}) must be greater than 0.", capacity));
    }
    m_transportCapacity = capacity;
  }

  void Edge::setAngle(std::pair<double, double> srcNodeCoordinates,
                      std::pair<double, double> dstNodeCoordinates) {
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

  void Edge::setGeometry(std::vector<std::pair<double, double>> geometry) {
    m_geometry = std::move(geometry);
  }

  Id Edge::id() const { return m_id; }
  Id Edge::source() const { return m_nodePair.first; }
  Id Edge::target() const { return m_nodePair.second; }
  std::pair<Id, Id> const& Edge::nodePair() const { return m_nodePair; }
  int Edge::capacity() const { return m_capacity; }
  int Edge::transportCapacity() const { return m_transportCapacity; }
  double Edge::angle() const { return m_angle; }
  std::vector<std::pair<double, double>> const& Edge::geometry() const {
    return m_geometry;
  }

  double Edge::deltaAngle(double const previousEdgeAngle) const {
    double deltaAngle{this->m_angle - previousEdgeAngle};
    if (deltaAngle > std::numbers::pi) {
      deltaAngle -= 2 * std::numbers::pi;
    } else if (deltaAngle < -std::numbers::pi) {
      deltaAngle += 2 * std::numbers::pi;
    }
    return deltaAngle;
  }
};  // namespace dsm