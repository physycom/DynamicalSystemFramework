#pragma once

#include "../geometry/Point.hpp"

#include <map>
#include <optional>

namespace dsf::mdt {
  class PointsCluster {
  private:
    std::map<std::time_t, dsf::geometry::Point> m_points;
    mutable std::optional<dsf::geometry::Point> m_centroid;

    void m_updateCentroid() const;

  public:
    PointsCluster() = default;

    void addPoint(std::time_t timestamp, dsf::geometry::Point const& point);

    dsf::geometry::Point centroid() const;

    inline std::size_t size() const noexcept { return m_points.size(); }
    inline bool empty() const noexcept { return m_points.empty(); }

    inline std::time_t firstTimestamp() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no first timestamp available.");
      }
      return m_points.begin()->first;
    }
    inline std::time_t lastTimestamp() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no last timestamp available.");
      }
      return m_points.rbegin()->first;
    }
    inline dsf::geometry::Point firstPoint() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no first point available.");
      }
      return m_points.begin()->second;
    }
    inline dsf::geometry::Point lastPoint() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no last point available.");
      }
      return m_points.rbegin()->second;
    }
    inline std::time_t duration() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no duration available.");
      }
      return this->lastTimestamp() - this->firstTimestamp();
    }
  };
}  // namespace dsf::mdt