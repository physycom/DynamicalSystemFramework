#pragma once

#include "Trajectory.hpp"
#include "../utility/Typedef.hpp"

#include <string>
#include <unordered_map>

namespace dsf::mdt {
  class TrajectoryCollection {
  private:
    std::unordered_map<Id, Trajectory> m_trajectories;

  public:
    /// @brief Construct a TrajectoryCollection, optionally importing from a CSV file.
    /// @param fileName The path to the CSV file.
    TrajectoryCollection(std::string const& fileName = std::string());
    /// @brief Import trajectories from a CSV file.
    /// @param fileName The path to the CSV file.
    /// @param sep The character used to separate values in the CSV file.
    void import(std::string const& fileName, char const sep = ';');
    /// @brief Export clustered trajectories to a CSV file.
    /// @param fileName The path to the output CSV file.
    /// @param sep The character used to separate values in the CSV file.
    void to_csv(std::string const& fileName, char const sep = ';') const;
    /// @brief Filter all point trajectories to identify stop points based on clustering and speed
    /// criteria.
    /// @param min_points_per_trajectory The minimum number of points required for a trajectory to be considered valid.
    /// @param cluster_radius_km The radius (in kilometers) to use for clustering points.
    /// @param max_speed_kph The max allowed speed (in km/h) to consider a cluster as a stop point.
    void filter(std::size_t const min_points_per_trajectory, double const cluster_radius_km, double const max_speed_kph = 150.0);
  };
}  // namespace dsf::mdt