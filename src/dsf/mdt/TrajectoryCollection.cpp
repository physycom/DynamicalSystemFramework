#include "TrajectoryCollection.hpp"

#include <format>
#include <fstream>
#include <mutex>

#include <rapidcsv.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

namespace dsf::mdt {
  TrajectoryCollection::TrajectoryCollection(std::string const& fileName) {
    if (!fileName.empty()) {
      this->import(fileName);
    }
  }

  void TrajectoryCollection::import(std::string const& fileName, char const sep) {
    // constexpr auto HEADER_LINE = "uid timestamp lat lon";
    // std::ifstream file{fileName};
    // if (!file.is_open()) {
    //   throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    // }

    // // Read the CSV file with header: uid;timestamp;lon;lat
    // std::string line;
    // std::getline(file, line);  // Skip header line
    // // Assert that header is correct
    // if (line != HEADER_LINE) {
    //   throw std::runtime_error("Invalid CSV header in file \"" + fileName + "\". Expected: \"" +
    //                            HEADER_LINE + "\", got: \"" + line + "\"");
    // }
    // while (std::getline(file, line)) {
    //   std::istringstream ss(line);
    //   Id uid;
    //   std::time_t timestamp;
    //   double lat, lon;
    //   // Data are separated by "\t" so just read with "\t" delimiter
    //   ss >> uid >> timestamp >> lat >> lon;
    //   m_trajectories[uid].addPoint(timestamp, dsf::geometry::Point(lon, lat));
    // }

    rapidcsv::Document doc(fileName, rapidcsv::LabelParams(0, -1), rapidcsv::SeparatorParams(sep));
    auto const& uids = doc.GetColumn<Id>("uid");
    auto const& timestamps = doc.GetColumn<std::time_t>("timestamp");
    auto const& lats = doc.GetColumn<double>("lat");
    auto const& lons = doc.GetColumn<double>("lon");
    
    for (std::size_t i = 0; i < uids.size(); ++i) {
      m_trajectories[uids[i]].addPoint(timestamps[i], dsf::geometry::Point(lons[i], lats[i]));
    }

    // tbb::parallel_for(std::size_t(0), uids.size(), [&](std::size_t i){
    //   static std::mutex mtx;
    //   std::scoped_lock lock(mtx);
    //   m_trajectories[uids[i]].addPoint(timestamps[i], dsf::geometry::Point(lons[i], lats[i]));
    // });
  }

  void TrajectoryCollection::filter(double const clusterRadius, double const maxSpeed) {
    tbb::parallel_for_each(
        m_trajectories.begin(),
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

    auto const HEADER_LINE = std::format("uid{}lon{}lat{}timestamp_in{}timestamp_out\n", sep, sep, sep, sep);
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