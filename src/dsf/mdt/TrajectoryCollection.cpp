#include "TrajectoryCollection.hpp"

#include <format>
#include <fstream>

#include <rapidcsv.h>

#include <tbb/parallel_for_each.h>

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
      m_trajectories[uids[i]].addPoint(timestamps[i],
                                       dsf::geometry::Point(lons[i], lats[i]));
    }
  }

  void TrajectoryCollection::filter(double const clusterRadius, double const maxSpeed) {
    tbb::parallel_for_each(m_trajectories.begin(),
                           m_trajectories.end(),
                           [this, clusterRadius, maxSpeed](auto& pair) {
                             pair.second.filter(clusterRadius, maxSpeed);
                           });
  }
  void TrajectoryCollection::to_csv(std::string const& fileName, char const sep) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    auto const HEADER_LINE =
        std::format("uid{}lon{}lat{}timestamp_in{}timestamp_out\n", sep, sep, sep, sep);
    // Write CSV header
    file << HEADER_LINE;

    for (auto const& [uid, trajectory] : m_trajectories) {
      for (auto const& cluster : trajectory.points()) {
        auto const centroid = cluster.centroid();
        file << uid << sep << centroid.x() << sep << centroid.y() << sep
             << cluster.firstTimestamp() << sep << cluster.lastTimestamp() << "\n";
      }
    }
  }
}  // namespace dsf::mdt