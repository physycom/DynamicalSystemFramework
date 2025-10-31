#include "TrajectoryCollection.hpp"

#include <format>
#include <fstream>

#include <rapidcsv.h>
#include <spdlog/spdlog.h>

#include <tbb/parallel_for_each.h>
#include <tbb/concurrent_set.h>

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(
      std::unordered_map<std::string,
                         std::variant<std::vector<Id>,
                                      std::vector<std::time_t>,
                                      std::vector<double>>>&& dataframe) {
    auto const& uids =
        std::get<std::vector<Id>>(dataframe.at("uid"));
    auto const& timestamps =
        std::get<std::vector<std::time_t>>(dataframe.at("timestamp"));
    auto const& lats =
        std::get<std::vector<double>>(dataframe.at("lat"));
    auto const& lons =
        std::get<std::vector<double>>(dataframe.at("lon"));

    for (std::size_t i = 0; i < uids.size(); ++i) {
      if (m_trajectories.find(uids[i]) == m_trajectories.end()) {
        m_trajectories[uids[i]] = std::vector<Trajectory>{};
        m_trajectories[uids[i]].emplace_back();
      }
      m_trajectories[uids[i]][0].addPoint(timestamps[i],
                                          dsf::geometry::Point(lons[i], lats[i]));
    }
  }

  TrajectoryCollection::TrajectoryCollection(std::string const& fileName) {
    if (!fileName.empty()) {
      this->import(fileName);
    }
  }

  void TrajectoryCollection::import(std::string const& fileName, char const sep) {
    rapidcsv::Document doc(
        fileName, rapidcsv::LabelParams(0, -1), rapidcsv::SeparatorParams(sep));
    
    std::unordered_map<std::string,
                           std::variant<std::vector<Id>,
                                        std::vector<std::time_t>,
                                        std::vector<double>>> dataframe;
    dataframe["uid"] = doc.GetColumn<Id>("uid");
    dataframe["timestamp"] = doc.GetColumn<std::time_t>("timestamp");
    dataframe["lat"] = doc.GetColumn<double>("lat");
    dataframe["lon"] = doc.GetColumn<double>("lon");
    *this = TrajectoryCollection(std::move(dataframe));
  }

  void TrajectoryCollection::filter(double const cluster_radius_km,
                                    double const max_speed_kph,
                                    std::size_t const min_points_per_trajectory,
                                    std::optional<std::time_t> const min_duration_min) {
    // Collect IDs to remove in parallel
    tbb::concurrent_set<Id> to_remove;
    tbb::concurrent_set<Id> to_split;

    tbb::parallel_for_each(
        m_trajectories.begin(),
        m_trajectories.end(),
        [&to_remove,
         &to_split,
         min_points_per_trajectory,
         cluster_radius_km,
         max_speed_kph,
         min_duration_min](auto& pair) {
          auto const& uid = pair.first;
          auto& trajectory =
              pair.second
                  [0];  // By now, each trajectory has only one segment as they were not split yet
          if (min_points_per_trajectory > 0 &&
              trajectory.size() < min_points_per_trajectory) {
            to_remove.insert(uid);
            return;
          }
          trajectory.filter(cluster_radius_km, max_speed_kph);
          if (min_points_per_trajectory > 0 &&
              trajectory.size() < min_points_per_trajectory) {
            to_remove.insert(uid);
            return;
          }
          if (!min_duration_min.has_value()) {
            return;
          }
          for (auto const& cluster : trajectory.points()) {
            if (cluster.duration() < min_duration_min.value() * 60) {
              to_split.insert(uid);
              return;
            }
          }
        });

    // Remove trajectories sequentially (fast for unordered_map)
    spdlog::info(
        "Removing {} ({:.2f}%) trajectories that do not meet the minimum points "
        "requirement after filtering.",
        to_remove.size(),
        (to_remove.size() * 100.0 / m_trajectories.size()));
    std::erase_if(m_trajectories, [&to_remove](auto const& pair) {
      return to_remove.contains(pair.first);
    });

    spdlog::info("Splitting {} trajectories based on minimum duration requirement.",
                 to_split.size());
    for (auto const& uid : to_split) {
      // Extract the trajectory
      if (!m_trajectories.contains(uid)) {
        continue;
      }
      auto& trajectories = m_trajectories.at(uid);
      auto originalTrajectory = std::move(trajectories[0]);
      trajectories.clear();

      Trajectory newTrajectory;
      for (auto const& cluster : originalTrajectory.points()) {
        newTrajectory.addCluster(cluster);
        if (cluster.duration() < min_duration_min.value() * 60) {
          continue;
        }
        // Cluster meets minimum duration - finalize current trajectory and start a new one
        if (!newTrajectory.empty()) {
          trajectories.emplace_back(std::move(newTrajectory));
          newTrajectory = Trajectory();
          newTrajectory.addCluster(cluster);
        }
      }
      if (newTrajectory.size() > 1) {
        trajectories.emplace_back(std::move(newTrajectory));
      }
    }
  }
  void TrajectoryCollection::to_csv(std::string const& fileName, char const sep) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    auto const HEADER_LINE = std::format(
        "uid{}segment{}lon{}lat{}timestamp_in{}timestamp_out\n", sep, sep, sep, sep, sep);
    // Write CSV header
    file << HEADER_LINE;

    for (auto const& [uid, trajectories] : m_trajectories) {
      std::size_t trajIdx = 0;
      for (auto const& trajectory : trajectories) {
        for (auto const& cluster : trajectory.points()) {
          auto const centroid = cluster.centroid();
          file << uid << sep << trajIdx << sep << centroid.x() << sep << centroid.y()
               << sep << cluster.firstTimestamp() << sep << cluster.lastTimestamp()
               << "\n";
        }
        ++trajIdx;
      }
    }
  }
}  // namespace dsf::mdt