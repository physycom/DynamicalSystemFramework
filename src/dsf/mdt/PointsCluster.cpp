#include "PointsCluster.hpp"

#include <algorithm>
#include <vector>

#include <tbb/parallel_sort.h>

namespace dsf::mdt {
  void PointsCluster::m_updateCentroid() const {
    if (m_points.empty()) {
      throw std::runtime_error("Cannot compute centroid of an empty PointsCluster.");
    }
    if (m_points.size() == 1) {
      m_centroid = m_points.begin()->point;
      m_timestampCentroid = m_points.begin()->timestamp;
      return;
    }
    // Ensure points are sorted by timestamp
    this->sort();

    // Collect x and y coordinates
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<std::time_t> ts;
    xs.reserve(m_points.size());
    ys.reserve(m_points.size());
    ts.reserve(m_points.size());
    for (auto const& activityPoint : m_points) {
      xs.push_back(activityPoint.point.x());
      ys.push_back(activityPoint.point.y());
      ts.push_back(activityPoint.timestamp);
    }

    // Helper to compute median; take vector by reference so we can modify it in place
    auto compute_median = []<typename T>(std::vector<T> v) -> double {
      auto const n = v.size();
      auto const mid = static_cast<std::size_t>(n * 0.5);

      std::nth_element(v.begin(), v.begin() + mid, v.end());
      auto const high = static_cast<double>(v[mid]);
      if (n % 2 == 1) {
        return high;
      }
      // even: need the lower middle as well
      std::nth_element(v.begin(), v.begin() + (mid - 1), v.end());
      auto const low = static_cast<double>(v[mid - 1]);
      return low  + ((high - low) * 0.5);
    };

    m_centroid = dsf::geometry::Point(compute_median(xs), compute_median(ys));
    m_timestampCentroid = compute_median(ts);
  }

  void PointsCluster::addActivityPoint(ActivityPoint const& activityPoint) noexcept {
    m_points.emplace_back(activityPoint);
    m_bSorted = false;
    m_centroid.reset();
    m_timestampCentroid.reset();
  }
  void PointsCluster::addPoint(std::time_t timestamp,
                               dsf::geometry::Point const& point) noexcept {
    this->addActivityPoint(ActivityPoint{timestamp, point});
  }
  void PointsCluster::sort() const noexcept {
    if (m_bSorted) {
      return;
    }
    tbb::parallel_sort(m_points.begin(),
                       m_points.end(),
                       [](ActivityPoint const& a, ActivityPoint const& b) {
                         return a.timestamp < b.timestamp;
                       });
    m_bSorted = true;
  }

  dsf::geometry::Point PointsCluster::centroid() const {
    if (!m_centroid.has_value()) {
      this->m_updateCentroid();
    }
    return *m_centroid;
  }
  double PointsCluster::timestampCentroid() const {
    if (!m_timestampCentroid.has_value()) {
      this->m_updateCentroid();
    }
    return *m_timestampCentroid;
  }

  std::time_t PointsCluster::firstTimestamp() const {
    if (m_points.empty()) {
      throw std::runtime_error("PointsCluster is empty, no first timestamp available.");
    }
    this->sort();
    return m_points.begin()->timestamp;
  }
  std::time_t PointsCluster::lastTimestamp() const {
    if (m_points.empty()) {
      throw std::runtime_error("PointsCluster is empty, no last timestamp available.");
    }
    this->sort();
    return m_points.rbegin()->timestamp;
  }
  std::time_t PointsCluster::duration() const {
    if (m_points.empty()) {
      throw std::runtime_error("PointsCluster is empty, no duration available.");
    }
    if (m_points.size() == 1) {
      return 0;
    }
    return this->lastTimestamp() - this->firstTimestamp();
  }
}  // namespace dsf::mdt