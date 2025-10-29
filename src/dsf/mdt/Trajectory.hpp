#pragma once

#include "PointsCluster.hpp"

#include <cmath>
#include <numbers>
#include <type_traits>
#include <vector>

#include <spdlog/spdlog.h>

namespace dsf::mdt {
  class Trajectory {
  private:
    std::vector<dsf::mdt::PointsCluster> m_points;

  public:
    Trajectory() = default;

    void addPoint(std::time_t timestamp, dsf::geometry::Point const& point);

    /// @brief Filter the trajectory to identify stop points based on clustering and speed criteria.
    /// @param clusterRadius The radius (in meters) to use for clustering points.
    /// @param maxSpeed The max allowed speed (in km/h) to consider a cluster as a stop point.
    /// @return A filtered trajectory with identified stop points.
  void filter(double const clusterRadius, double const maxSpeed = 150.0);

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
}  // namespace dsf::mdt