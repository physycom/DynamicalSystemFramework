#pragma once

#include "Trajectory.hpp"
#include "../utility/Typedef.hpp"

#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace dsf::mdt {
  class TrajectoryCollection {
  private:
    std::unordered_map<Id, std::vector<Trajectory>> m_trajectories;

  public:
    TrajectoryCollection(
        std::unordered_map<std::string,
                           std::variant<std::vector<Id>,
                                        std::vector<std::time_t>,
                                        std::vector<double>>>&& dataframe);
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
    /// @param cluster_radius_km The radius (in kilometers) to use for clustering points.
    /// @param max_speed_kph The max allowed speed (in km/h) to consider a cluster as a stop point. Default is 150.0 km/h.
    /// @param min_points_per_trajectory The minimum number of points required for a trajectory to be considered valid. Default is 2.
    /// @param min_duration_min The minimum duration (in minutes) for a cluster to be considered a stop point.
    /// If stops are detected, trajectories may be split into multiple segments.
    void filter(double const cluster_radius_km,
                double const max_speed_kph = 150.0,
                std::size_t const min_points_per_trajectory = 2,
                std::optional<std::time_t> const min_duration_min = std::nullopt);
    /// @brief Get the underlying trajectories map.
    /// @return A const reference to the map of trajectories.
    inline auto const& trajectories() const noexcept { return m_trajectories; }
  };
}  // namespace dsf::mdt