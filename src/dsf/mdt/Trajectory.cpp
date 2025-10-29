#include "Trajectory.hpp"

namespace dsf::mdt {
  void Trajectory::addPoint(std::time_t timestamp, dsf::geometry::Point const& point) {
    // Create a new PointsCluster containing the single activity point and add it
    PointsCluster cluster;
    cluster.addPoint(ActivityPoint{timestamp, point});
    m_points.push_back(std::move(cluster));
  }

  void Trajectory::filter(double const clusterRadius,
                          double const maxSpeed) {
    auto rawPoints = std::move(m_points);
    if (rawPoints.empty()) {
      return;
    }

    constexpr double KMH_TO_MS = 1000.0 / 3600.0;  // Convert km/h to m/s
    double const maxSpeedMS =
        maxSpeed * KMH_TO_MS;  // maxSpeed is in km/h, convert to m/s

    std::vector<PointsCluster> clusterCandidates;

    auto it = rawPoints.begin();
    PointsCluster currentCluster;
    currentCluster.addPoint(ActivityPoint{it->firstTimestamp(), it->centroid()});

    ++it;
    for (; it != rawPoints.end(); ++it) {
      std::time_t timestamp = it->firstTimestamp();
      dsf::geometry::Point const& point = it->centroid();

      // Compute distance from current point to cluster centroid
      double const distance =
          dsf::geometry::haversine_m(currentCluster.centroid(), point);

      if (distance < clusterRadius) {
        // Add point to current cluster
        currentCluster.addPoint(ActivityPoint{timestamp, point});
      } else {
        // Distance exceeds threshold - finalize current cluster and start new one
        if (!currentCluster.empty()) {
          clusterCandidates.push_back(currentCluster);
        }

        // Start new cluster with current point
        currentCluster = PointsCluster();
        currentCluster.addPoint(ActivityPoint{timestamp, point});
      }
    }

    // Handle the last cluster
    if (!currentCluster.empty()) {
      clusterCandidates.push_back(currentCluster);
    }

    // Apply speed filtering: only keep clusters with average speed below maxSpeed
    // Speed is computed from first to last point in the cluster
    for (auto cluster : clusterCandidates) {
      if (cluster.size() < 2) {
        // Single point cluster - always consider it a stop point (speed = 0)
        m_points.push_back(cluster);
        continue;
      }
      cluster.sort();

      // Compute average speed for this cluster
      // Speed = distance / time (from first to last point in cluster)
      std::time_t const duration = cluster.duration();  // in seconds

      if (duration > 0) {
        // Calculate distance traveled from first to last point
        dsf::geometry::Point const firstPt = cluster.points().front().point;
        dsf::geometry::Point const lastPt = cluster.points().back().point;
        double const distanceTraveled =
            dsf::geometry::haversine_m(firstPt, lastPt);  // in meters

        // Compute average speed in m/s
        double const avgSpeedMS = distanceTraveled / static_cast<double>(duration);

        // Only add clusters where average speed is below maxSpeed threshold
        // These represent stop points or slow-moving areas
        if (avgSpeedMS < maxSpeedMS) {
            m_points.push_back(cluster);
        }
      } else {
        // Duration is 0 - all points have same timestamp, treat as stopped
        m_points.push_back(cluster);
      }
    }
    }
}  // namespace dsf::mdt