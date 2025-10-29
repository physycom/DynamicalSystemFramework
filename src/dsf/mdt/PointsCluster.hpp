#pragma once

#include "../geometry/Point.hpp"

#include <vector>

namespace dsf::mdt {
  struct ActivityPoint {
    std::time_t timestamp;
    dsf::geometry::Point point;
  };

  class PointsCluster {
  private:
    mutable std::vector<ActivityPoint> m_points;
    mutable std::optional<dsf::geometry::Point> m_centroid;
    mutable bool m_bSorted;

    void m_updateCentroid() const;

  public:
    PointsCluster() = default;

    void addPoint(ActivityPoint const& activityPoint);

    void sort() const;

    dsf::geometry::Point centroid() const;

    inline std::size_t size() const noexcept { return m_points.size(); }
    inline bool empty() const noexcept { return m_points.empty(); }
    inline std::vector<ActivityPoint> const& points() const noexcept { return m_points; }

    inline std::time_t firstTimestamp() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no first timestamp available.");
      }
      this->sort();
      return m_points.begin()->timestamp;
    }
    inline std::time_t lastTimestamp() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no last timestamp available.");
      }
      this->sort();
      return m_points.rbegin()->timestamp;
    }
    inline std::time_t duration() const {
      if (m_points.empty()) {
        throw std::runtime_error("PointsCluster is empty, no duration available.");
      }
      if (m_points.size() == 1) {
        return 0;
      }
      return this->lastTimestamp() - this->firstTimestamp();
    }
  };
}  // namespace dsf::mdt