#include "PointsCluster.hpp"
#include <algorithm>
#include <vector>

namespace dsf::mdt {
  void PointsCluster::m_updateCentroid() const {
    if (m_points.empty()) {
      throw std::runtime_error("Cannot compute centroid of an empty PointsCluster.");
    }
    // Collect x and y coordinates
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(m_points.size());
    ys.reserve(m_points.size());
    for (const auto& [timestamp, point] : m_points) {
      xs.push_back(point.x());
      ys.push_back(point.y());
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

  void PointsCluster::addPoint(std::time_t timestamp, dsf::geometry::Point const& point) {
    if (m_points.contains(timestamp)) {
      throw std::invalid_argument(std::format(
          "A point with the given timestamp ({}) already exists in the cluster.",
          timestamp));
    }
    m_points.emplace(timestamp, point);
  }

  dsf::geometry::Point PointsCluster::centroid() const {
    if (!m_centroid.has_value()) {
      this->m_updateCentroid();
    }
    return *m_centroid;
  }
}  // namespace dsf::mdt