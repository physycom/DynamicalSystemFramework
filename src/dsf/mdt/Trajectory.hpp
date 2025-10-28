#pragma once

#include "PointsCluster.hpp"

#include <cmath>
#include <numbers>
#include <type_traits>
#include <vector>

#include <spdlog/spdlog.h>

namespace dsf::mdt {
  template <typename Point_T>
    requires std::is_same_v<Point_T, dsf::geometry::Point> ||
             std::is_same_v<Point_T, dsf::mdt::PointsCluster>
  class Trajectory {
  private:
    std::map<std::time_t, Point_T> m_points;

  public:
    Trajectory() = default;

    void addPoint(std::time_t timestamp, Point_T const& point);

    /// @brief Filter the trajectory to identify stop points based on clustering and speed criteria.
    /// @tparam U Alias for Point_T to enable SFINAE.
    /// @param clusterRadius The radius (in meters) to use for clustering points.
    /// @param maxSpeed The max allowed speed (in km/h) to consider a cluster as a stop point.
    /// @return A filtered trajectory with identified stop points.
    template <typename U = Point_T>
    std::enable_if_t<std::is_same_v<U, dsf::geometry::Point>,
                     Trajectory<dsf::mdt::PointsCluster>>
    filter(double const clusterRadius, double const maxSpeed = 150.0) const;

    /// @brief Get the number of points in the trajectory.
    /// @return The size of the trajectory.
    inline std::size_t size() const noexcept { return m_points.size(); }
    /// @brief Check if the trajectory is empty.
    /// @return True if the trajectory has no points, false otherwise.
    inline bool empty() const noexcept { return m_points.empty(); }
    /// @brief Get the underlying points map.
    /// @return A const reference to the map of points.
    inline auto const& points() const noexcept { return m_points; }
  };

  template <typename Point_T>
    requires std::is_same_v<Point_T, dsf::geometry::Point> ||
             std::is_same_v<Point_T, dsf::mdt::PointsCluster>
  void Trajectory<Point_T>::addPoint(std::time_t timestamp, Point_T const& point) {
    auto [it, inserted] = m_points.try_emplace(timestamp, point);
    if (!inserted) {
        spdlog::warn("A point with the given timestamp ({}) already exists in the trajectory. Skipping addition.", timestamp);
    }
  }

  // Filter implementation for Trajectory<Point> -> Trajectory<PointsCluster>
  template <typename Point_T>
    requires std::is_same_v<Point_T, dsf::geometry::Point> ||
             std::is_same_v<Point_T, dsf::mdt::PointsCluster>
  template <typename U>
  std::enable_if_t<std::is_same_v<U, dsf::geometry::Point>,
                   Trajectory<dsf::mdt::PointsCluster>>
  Trajectory<Point_T>::filter(double const clusterRadius, double const maxSpeed) const {
    if (m_points.empty()) {
      return Trajectory<PointsCluster>();
    }

    constexpr double KMH_TO_MS = 1000.0 / 3600.0;  // Convert km/h to m/s
    double const maxSpeedMS =
        maxSpeed * KMH_TO_MS;  // maxSpeed is in km/h, convert to m/s

    Trajectory<PointsCluster> result;
    std::vector<PointsCluster> clusterCandidates;

    auto it = m_points.begin();
    PointsCluster currentCluster;
    currentCluster.addPoint(it->first, it->second);

    ++it;
    for (; it != m_points.end(); ++it) {
      std::time_t timestamp = it->first;
      dsf::geometry::Point const& point = it->second;

      // Compute distance from current point to cluster centroid
      double const distance =
          dsf::geometry::haversine_m(currentCluster.centroid(), point);

      if (distance < clusterRadius) {
        // Add point to current cluster
        currentCluster.addPoint(timestamp, point);
      } else {
        // Distance exceeds threshold - finalize current cluster and start new one
        if (!currentCluster.empty()) {
          clusterCandidates.push_back(currentCluster);
        }

        // Start new cluster with current point
        currentCluster = PointsCluster();
        currentCluster.addPoint(timestamp, point);
      }
    }

    // Handle the last cluster
    if (!currentCluster.empty()) {
      clusterCandidates.push_back(currentCluster);
    }

    // Apply speed filtering: only keep clusters with average speed below maxSpeed
    // Speed is computed from first to last point in the cluster
    for (auto const& cluster : clusterCandidates) {
      if (cluster.size() < 2) {
        // Single point cluster - always consider it a stop point (speed = 0)
        result.addPoint(cluster.firstTimestamp(), cluster);
        continue;
      }

      // Compute average speed for this cluster
      // Speed = distance / time (from first to last point in cluster)
      std::time_t const duration = cluster.duration();  // in seconds

      if (duration > 0) {
        // Calculate distance traveled from first to last point
        dsf::geometry::Point const firstPt = cluster.firstPoint();
        dsf::geometry::Point const lastPt = cluster.lastPoint();
        double const distanceTraveled =
            dsf::geometry::haversine_m(firstPt, lastPt);  // in meters

        // Compute average speed in m/s
        double const avgSpeedMS = distanceTraveled / static_cast<double>(duration);

        // Only add clusters where average speed is below maxSpeed threshold
        // These represent stop points or slow-moving areas
        if (avgSpeedMS < maxSpeedMS) {
          result.addPoint(cluster.firstTimestamp(), cluster);
        }
      } else {
        // Duration is 0 - all points have same timestamp, treat as stopped
        result.addPoint(cluster.firstTimestamp(), cluster);
      }
    }

    return result;
  }
}  // namespace dsf::mdt