#include "TrajectoryCollection.hpp"

#include <fstream>

#include <rapidcsv.h>
#include <tbb/parallel_for_each.h>

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(std::string const& fileName) {
    if (!fileName.empty()) {
      this->import(fileName);
    }
  }

  void TrajectoryCollection::import(std::string const& fileName) {
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }

    rapidcsv::Document csvReader(
        file, rapidcsv::LabelParams(0, -1), rapidcsv::SeparatorParams(';'));
    auto const rowCount = csvReader.GetRowCount();

    if (rowCount == 0) {
      return;  // Nothing to import
    }

    // Read all columns at once for better performance
    auto const uids = csvReader.GetColumn<std::string>("uid");
    auto const timestamps = csvReader.GetColumn<std::time_t>("timestamp");
    auto const lons = csvReader.GetColumn<double>("lon");
    auto const lats = csvReader.GetColumn<double>("lat");

    // Build trajectories by grouping consecutive rows with same uid
    std::string currentUid = uids[0];
    Trajectory<dsf::geometry::Point> trajectory;

    for (std::size_t i = 0; i < rowCount; ++i) {
      if (uids[i] != currentUid) {
        // Store the previous trajectory
        m_pointTrajectories.emplace(std::move(currentUid), std::move(trajectory));
        // Start a new trajectory
        currentUid = uids[i];
        trajectory = Trajectory<dsf::geometry::Point>();
      }

      dsf::geometry::Point point(lons[i], lats[i]);
      trajectory.addPoint(timestamps[i], point);
    }

    // Don't forget to store the last trajectory!
    if (!trajectory.empty()) {
      m_pointTrajectories.emplace(std::move(currentUid), std::move(trajectory));
    }
  }

  void TrajectoryCollection::filter(double const clusterRadius, double const maxSpeed) {
    tbb::parallel_for_each(
        m_pointTrajectories.cbegin(),
        m_pointTrajectories.cend(),
        [this, clusterRadius, maxSpeed](auto const& pair) {
          auto const& [uid, trajectory] = pair;
          auto filteredTrajectory = trajectory.filter(clusterRadius, maxSpeed);
          m_clusterTrajectories.emplace(uid, std::move(filteredTrajectory));
        });
  }
  void TrajectoryCollection::to_csv(std::string const& fileName) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    // Write CSV header
    file << "uid;lon;lat;timestamp_in;timestamp_out\n";

    for (auto const& [uid, trajectory] : m_clusterTrajectories) {
      for (auto const& [timestamp, cluster] : trajectory.points()) {
        auto const centroid = cluster.centroid();
        file << uid << ";" << centroid.x() << ";" << centroid.y() << ";"
             << cluster.firstTimestamp() << ";" << cluster.lastTimestamp() << "\n";
      }
    }
  }
}  // namespace dsf::mdt