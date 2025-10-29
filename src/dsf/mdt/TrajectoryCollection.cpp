#include "TrajectoryCollection.hpp"

#include <fstream>

#include <tbb/parallel_for_each.h>

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(std::string const& fileName) {
    if (!fileName.empty()) {
      this->import(fileName);
    }
  }

  void TrajectoryCollection::import(std::string const& fileName) {
    constexpr auto HEADER_LINE = "uid timestamp lat lon";
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }

    // Read the CSV file with header: uid;timestamp;lon;lat
    std::string line;
    std::getline(file, line);  // Skip header line
    // Assert that header is correct
    if (line != HEADER_LINE) {
      throw std::runtime_error("Invalid CSV header in file \"" + fileName + "\". Expected: \"" +
                               HEADER_LINE + "\", got: \"" + line + "\"");
    }
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      Id uid;
      std::time_t timestamp;
      double lat, lon;
      // Data are separated by "\t" so just read with "\t" delimiter
      ss >> uid >> timestamp >> lat >> lon;
      m_trajectories[uid].addPoint(timestamp, dsf::geometry::Point(lon, lat));
    }
  }

  void TrajectoryCollection::filter(double const clusterRadius, double const maxSpeed) {
    tbb::parallel_for_each(
        m_trajectories.begin(),
        m_trajectories.end(),
        [this, clusterRadius, maxSpeed](auto& pair) {
          pair.second.filter(clusterRadius, maxSpeed);
        });
  }
  void TrajectoryCollection::to_csv(std::string const& fileName) const {
    std::ofstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }

    // Write CSV header
    file << "uid;lon;lat;timestamp_in;timestamp_out\n";

    for (auto const& [uid, trajectory] : m_trajectories) {
      for (auto const& cluster : trajectory.points()) {
        auto const centroid = cluster.centroid();
        file << uid << ";" << centroid.x() << ";" << centroid.y() << ";"
             << cluster.firstTimestamp() << ";" << cluster.lastTimestamp() << "\n";
      }
    }
  }
}  // namespace dsf::mdt