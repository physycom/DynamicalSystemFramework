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
    /// @param clusterRadius The radius (in meters) to use for clustering points.
    /// @param maxSpeed The max allowed speed (in km/h) to consider a cluster as a stop point.
    void filter(double const clusterRadius, double const maxSpeed = 150.0);
  };
}  // namespace dsf::mdt