#include "PointsCluster.hpp"
#include <algorithm>
#include <vector>

namespace dsf::mdt {
  void PointsCluster::m_updateCentroid() const {
    if (m_points.empty()) {
      throw std::runtime_error("Cannot compute centroid of an empty PointsCluster.");
    }
    if (m_points.size() == 1) {
      m_centroid = m_points.begin()->point;
      return;
    }
    // Collect x and y coordinates
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(m_points.size());
    ys.reserve(m_points.size());
    for (auto const& activityPoint : m_points) {
      xs.push_back(activityPoint.point.x());
      ys.push_back(activityPoint.point.y());
    }

    // Helper to compute median; take vector by value so we can modify it
    auto compute_median = [](std::vector<double> v) -> double {
      const size_t n = v.size();
      const size_t mid = n / 2;
      if (n == 0) {
        throw std::runtime_error("Cannot compute median of empty vector");
      }
      std::nth_element(v.begin(), v.begin() + mid, v.end());
      double high = v[mid];
      if (n % 2 == 1) {
        return high;
      }
      // even: need the lower middle as well
      std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
      double low = v[mid - 1];
      return (low + high) / 2.0;
    };

    m_centroid = dsf::geometry::Point(compute_median(xs), compute_median(ys));
  }

  void PointsCluster::addPoint(ActivityPoint const& activityPoint) {
    m_points.emplace_back(activityPoint);
    m_bSorted = false;
    m_centroid.reset();
  }
  void PointsCluster::sort() const {
    if (!m_bSorted) {
      std::sort(m_points.begin(), m_points.end(),
                [](ActivityPoint const& a, ActivityPoint const& b) {
                  return a.timestamp < b.timestamp;
                });
      m_bSorted = true;
    }
  }

  dsf::geometry::Point PointsCluster::centroid() const {
    if (!m_centroid.has_value()) {
      this->m_updateCentroid();
    }
    return *m_centroid;
  }
}  // namespace dsf::mdt