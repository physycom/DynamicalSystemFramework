#include "TrajectoryCollection.hpp"

#include <format>
#include <fstream>

#include <rapidcsv.h>
#include <spdlog/spdlog.h>

#include <tbb/parallel_for_each.h>
#include <tbb/concurrent_vector.h>

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(std::string const& fileName) {
    if (!fileName.empty()) {
      this->import(fileName);
    }
  }

  void TrajectoryCollection::import(std::string const& fileName, char const sep) {
    rapidcsv::Document doc(
        fileName, rapidcsv::LabelParams(0, -1), rapidcsv::SeparatorParams(sep));
    auto const& uids = doc.GetColumn<Id>("uid");
    auto const& timestamps = doc.GetColumn<std::time_t>("timestamp");
    auto const& lats = doc.GetColumn<double>("lat");
    auto const& lons = doc.GetColumn<double>("lon");

    for (std::size_t i = 0; i < uids.size(); ++i) {
      if (m_trajectories.find(uids[i]) == m_trajectories.end()) {
        m_trajectories[uids[i]] = std::vector<Trajectory>{};
        m_trajectories[uids[i]].emplace_back();
      }
      m_trajectories[uids[i]][0].addPoint(timestamps[i],
                                       dsf::geometry::Point(lons[i], lats[i]));
    }
  }

  void TrajectoryCollection::filter(double const cluster_radius_km, double const max_speed_kph, std::size_t const min_points_per_trajectory, std::optional<std::time_t> const min_duration_min) {
    // Collect IDs to remove in parallel
    tbb::concurrent_vector<Id> to_remove;
    tbb::concurrent_vector<Id> to_split;
    
    tbb::parallel_for_each(m_trajectories.begin(),
                           m_trajectories.end(),
                           [&to_remove, &to_split, min_points_per_trajectory, cluster_radius_km, max_speed_kph, min_duration_min](auto& pair) {
                             auto& trajectory = pair.second[0];
                             if (min_points_per_trajectory > 0 && trajectory.size() < min_points_per_trajectory) {
                               to_remove.push_back(pair.first);
                               return;
                             }
                             // By now, each trajectory has only one segment as they
                             // were not split yet
                             trajectory.filter(cluster_radius_km, max_speed_kph);
                             if (min_points_per_trajectory > 0 && trajectory.size() < min_points_per_trajectory) {
                               to_remove.push_back(pair.first);
                               return;
                             }
                             if (!min_duration_min.has_value()) {
                               return;
                             }
                             for (auto const& cluster : trajectory.points()) {
                               if (cluster.duration() < min_duration_min.value() * 60) {
                                 to_split.push_back(pair.first);
                                 return;
                               }
                             }
                           });
    
    // Remove trajectories sequentially (fast for unordered_map)
    spdlog::debug("Removing {} ({}%) trajectories that do not meet the minimum points requirement after filtering.", to_remove.size(), (to_remove.size() * 100.0 / m_trajectories.size()));
    for (auto const& trajIdx : to_remove) {
      if(!m_trajectories.erase(trajIdx)) {
        throw std::runtime_error("Failed to erase trajectory with ID " + std::to_string(trajIdx));
      }
    }

    spdlog::debug("Splitting {} trajectories based on minimum duration requirement.", to_split.size());
    for (auto const& trajIdx : to_split) {
      // Extract the trajectory
      if (!m_trajectories.contains(trajIdx)) {
        continue;
      }
      auto& trajectories = m_trajectories[trajIdx];
      auto originalTrajectory = std::move(trajectories[0]);
      // Clear existing trajectories
      trajectories.clear();

      Trajectory newTrajectory;
      for (auto const& cluster : originalTrajectory.points()) {
        auto copy = cluster;
        newTrajectory.addCluster(copy);
        if (cluster.duration() < min_duration_min.value() * 60) {
          continue;
        }
        // Cluster meets minimum duration - finalize current trajectory and start a new one
        if (!newTrajectory.empty()) {
          trajectories.emplace_back(std::move(newTrajectory));
        }
      }
    }
  }
  void TrajectoryCollection::to_csv(std::string const& fileName, char const sep) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    auto const HEADER_LINE =
        std::format("uid{}segment{}lon{}lat{}timestamp_in{}timestamp_out\n", sep, sep, sep, sep, sep);
    // Write CSV header
    file << HEADER_LINE;

    for (auto const& [uid, trajectories] : m_trajectories) {
      std::size_t trajIdx = 0;
      for (auto const& trajectory : trajectories) {
        for (auto const& cluster : trajectory.points()) {
          auto const centroid = cluster.centroid();
          file << uid << sep << trajIdx << sep << centroid.x() << sep << centroid.y() << sep
              << cluster.firstTimestamp() << sep << cluster.lastTimestamp() << "\n";
        }
        ++trajIdx;
      }
    }
  }
}  // namespace dsf::mdt