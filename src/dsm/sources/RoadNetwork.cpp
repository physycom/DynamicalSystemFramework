
#include "../headers/RoadNetwork.hpp"

#include <algorithm>

namespace dsm {
  void RoadNetwork::m_addMissingNodes(Id const nodeId) {
    auto const srcNodeId{m_edges.at(nodeId)->source()};
    auto const dstNodeId{m_edges.at(nodeId)->target()};
    if (srcNodeId < m_adjacencyMatrix.n() && dstNodeId < m_adjacencyMatrix.n()) {
      if (!m_adjacencyMatrix.contains(srcNodeId, dstNodeId)) {
        m_adjacencyMatrix.insert(srcNodeId, dstNodeId);
      }
    } else {
      m_adjacencyMatrix.insert(srcNodeId, dstNodeId);
    }
    if (!m_nodes.contains(srcNodeId)) {
      m_nodes.emplace(srcNodeId, std::make_unique<Intersection>(srcNodeId));
    }
    if (!m_nodes.contains(dstNodeId)) {
      m_nodes.emplace(dstNodeId, std::make_unique<Intersection>(dstNodeId));
    }
  }
  RoadNetwork::RoadNetwork()
      : Network{AdjacencyMatrix()},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {}

  RoadNetwork::RoadNetwork(AdjacencyMatrix const& adj)
      : Network{adj},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {}

  RoadNetwork::RoadNetwork(
      const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet)
      : Network{AdjacencyMatrix(streetSet)} {
    // for (auto& street : streetSet) {
    //   m_edges.emplace(street.second->id(), street.second.get());

    //   Id node1 = street.second->nodePair().first;
    //   Id node2 = street.second->nodePair().second;
    //   m_nodes.emplace(node1, std::make_unique<Intersection>(node1));
    //   m_nodes.emplace(node2, std::make_unique<Intersection>(node2));
    // }

    // buildAdj();
  }

  void RoadNetwork::m_reassignIds() {
    // not sure about this, might need a bit more work
    auto const oldStreetSet{std::move(m_edges)};
    auto const N{nNodes()};
    std::unordered_map<Id, Id> newStreetIds;
    for (auto& [streetId, street] : oldStreetSet) {
      const auto srcId{street->source()};
      const auto dstId{street->target()};
      const auto newStreetId{static_cast<Id>(srcId * N + dstId)};
      assert(!m_edges.contains(newStreetId));
      street->resetId(newStreetId);
      if (street->isSpire() && street->isStochastic()) {
        addEdge(std::move(dynamic_cast<StochasticSpireStreet&>(*street)));
      } else if (street->isStochastic()) {
        addEdge(std::move(dynamic_cast<StochasticStreet&>(*street)));
      } else if (street->isSpire()) {
        addEdge(std::move(dynamic_cast<SpireStreet&>(*street)));
      } else {
        addEdge(std::move(*street));
      }
      newStreetIds.emplace(streetId, newStreetId);
    }
    for (const auto& [nodeId, node] : m_nodes) {
      // This is probably not the best way to do this
      if (node->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*node);
        const auto& oldStreetPriorities{intersection.streetPriorities()};
        std::set<Id> newStreetPriorities;
        for (const auto streetId : oldStreetPriorities) {
          newStreetPriorities.emplace(newStreetIds.at(streetId));
        }
        intersection.setStreetPriorities(newStreetPriorities);
      }
      if (node->isTrafficLight()) {
        auto& trafficLight = dynamic_cast<TrafficLight&>(*node);
        std::unordered_map<Id, std::vector<TrafficLightCycle>> newCycles;
        for (auto const& [streetId, cycles] : trafficLight.cycles()) {
          newCycles.emplace(newStreetIds.at(streetId), std::move(cycles));
        }
        trafficLight.setCycles(newCycles);
      }
    }
  }

  void RoadNetwork::initTrafficLights() {
    for (auto const& [id, pNode] : m_nodes) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);
      if (!tl.streetPriorities().empty() || !tl.cycles().empty()) {
        continue;
      }
      auto const& inNeighbours = m_adjacencyMatrix.getCol(id);
      std::map<Id, int, std::greater<int>> capacities;
      std::unordered_map<Id, double> streetAngles;
      std::unordered_map<Id, double> maxSpeeds;
      std::unordered_map<Id, int> nLanes;
      double higherSpeed{0.}, lowerSpeed{std::numeric_limits<double>::max()};
      int higherNLanes{0}, lowerNLanes{std::numeric_limits<int>::max()};
      if (inNeighbours.size() < 3) {
        Logger::warning(std::format(
            "Not enough in neighbours {} for Traffic Light {}", inNeighbours.size(), id));
        // Replace with a normal intersection
        auto const& coordinates{pNode->coords()};
        if (coordinates.has_value()) {
          m_nodes[id] = std::make_unique<Intersection>(id, *coordinates);
        } else {
          m_nodes[id] = std::make_unique<Intersection>(id);
        }
        continue;
      }
      for (auto const& inId : inNeighbours) {
        auto const streetId{inId * nNodes() + id};
        auto const& pStreet{m_edges.at(streetId)};

        double const speed{pStreet->maxSpeed()};
        int const nLan{pStreet->nLanes()};
        auto const cap{pStreet->capacity()};
        Logger::debug(std::format("Street {} with capacity {}", streetId, cap));
        capacities.emplace(streetId, cap);
        streetAngles.emplace(streetId, pStreet->angle());

        maxSpeeds.emplace(streetId, speed);
        nLanes.emplace(streetId, nLan);

        higherSpeed = std::max(higherSpeed, speed);
        lowerSpeed = std::min(lowerSpeed, speed);

        higherNLanes = std::max(higherNLanes, nLan);
        lowerNLanes = std::min(lowerNLanes, nLan);
      }
      if (higherSpeed != lowerSpeed) {
        // Assign streets with higher speed to priority
        for (auto const& [sid, speed] : maxSpeeds) {
          if (speed == higherSpeed) {
            tl.addStreetPriority(sid);
          }
        }
        // continue;
      } else if (higherNLanes != lowerNLanes) {
        for (auto const& [sid, nLan] : nLanes) {
          if (nLan == higherNLanes) {
            tl.addStreetPriority(sid);
          }
        }
        // continue;
      }
      // Set first two elements of capacities to street priorities
      // auto it{capacities.begin()};
      // tl.addStreetPriority(it->first);
      // ++it;
      // if (it != capacities.end()) {
      //   tl.addStreetPriority(it->first);
      //   continue;
      // }
      // Id firstStreetId{streetAngles.begin()->first};
      // tl.addStreetPriority(firstStreetId);
      // for (auto const& [streetId, angle] : streetAngles) {
      //   if (streetId == firstStreetId) {
      //     continue;
      //   }
      //   if (angle == streetAngles.begin()->second) {
      //     tl.addStreetPriority(streetId);
      //   }
      // }
      // if (tl.streetPriorities().size() > 1) {
      //   continue;
      // }
      if (tl.streetPriorities().empty()) {
        Logger::warning(std::format("Failed to auto-init Traffic Light {}", id));
        continue;
      }

      // Assign cycles
      std::pair<Delay, Delay> greenTimes;
      {
        auto capPriority{0.}, capNoPriority{0.};
        std::unordered_map<Id, double> normCapacities;
        auto sum{0.};
        for (auto const& [streetId, cap] : capacities) {
          sum += cap;
        }
        for (auto const& [streetId, cap] : capacities) {
          normCapacities.emplace(streetId, cap / sum);
        }
        for (auto const& [streetId, normCap] : normCapacities) {
          if (tl.streetPriorities().contains(streetId)) {
            capPriority += normCap;
          } else {
            capNoPriority += normCap;
          }
        }
        Logger::debug(
            std::format("Capacities for Traffic Light {}: priority {} no priority {}",
                        id,
                        capPriority,
                        capNoPriority));
        greenTimes = std::make_pair(static_cast<Delay>(capPriority * tl.cycleTime()),
                                    static_cast<Delay>(capNoPriority * tl.cycleTime()));
      }
      std::for_each(inNeighbours.begin(), inNeighbours.end(), [&](Id const inId) {
        auto const streetId{inId * nNodes() + id};
        auto const nLane{nLanes.at(streetId)};
        Delay greenTime{greenTimes.first};
        Delay phase{0};
        if (!tl.streetPriorities().contains(streetId)) {
          phase = greenTime;
          greenTime = greenTimes.second;
        }
        Logger::debug(
            std::format("Setting cycle for street {} with green time {} and phase {}",
                        streetId,
                        greenTime,
                        phase));
        switch (nLane) {
          case 3:
            tl.setCycle(streetId,
                        dsm::Direction::RIGHTANDSTRAIGHT,
                        TrafficLightCycle{static_cast<Delay>(greenTime * 2. / 3), phase});
            tl.setCycle(
                streetId,
                dsm::Direction::LEFT,
                TrafficLightCycle{
                    static_cast<Delay>(greenTime / 3.),
                    static_cast<Delay>(phase + static_cast<Delay>(greenTime * 2. / 3))});
            break;
          default:
            tl.setCycle(
                streetId, dsm::Direction::ANY, TrafficLightCycle{greenTime, phase});
            break;
        }
      });
    }
  }

  void RoadNetwork::buildAdj() {
    // find max values in streets node pairs
    m_maxAgentCapacity = 0;
    for (auto const& [streetId, pStreet] : m_edges) {
      m_maxAgentCapacity += pStreet->capacity();
      if (pStreet->geometry().empty()) {
        std::vector<std::pair<double, double>> coords;
        auto pair{m_nodes.at(pStreet->source())->coords()};
        if (pair.has_value()) {
          coords.emplace_back(pair.value().second, pair.value().first);
        }
        pair = m_nodes.at(pStreet->target())->coords();
        if (pair.has_value()) {
          coords.emplace_back(pair.value().second, pair.value().first);
        }
        pStreet->setGeometry(coords);
      }
    }
    this->m_reassignIds();
  }

  void RoadNetwork::adjustNodeCapacities() {
    double value;
    for (Id nodeId = 0; nodeId < m_nodes.size(); ++nodeId) {
      value = 0.;
      auto const N{nNodes()};
      for (const auto& sourceId : m_adjacencyMatrix.getCol(nodeId)) {
        auto const streetId{sourceId * N + nodeId};
        auto const& pStreet{m_edges.at(streetId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      auto const& pNode{m_nodes.at(nodeId)};
      pNode->setCapacity(value);
      value = 0.;
      for (const auto& targetId : m_adjacencyMatrix.getRow(nodeId)) {
        auto const streetId{nodeId * N + targetId};
        auto const& pStreet{m_edges.at(streetId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      pNode->setTransportCapacity(value == 0. ? 1. : value);
      if (pNode->capacity() == 0) {
        pNode->setCapacity(value);
      }
    }
  }

  void RoadNetwork::importMatrix(const std::string& fileName,
                                 bool isAdj,
                                 double defaultSpeed) {
    // check the file extension
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (fileExt == "dsm") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Cannot find file: " + fileName));
      }
      Size rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Adjacency matrix must be square"));
      }
      Size n{rows};
      // each line has 2 elements
      while (!file.eof()) {
        Id index;
        double val;
        file >> index >> val;
        const auto srcId{static_cast<Id>(index / n)};
        const auto dstId{static_cast<Id>(index % n)};
        if (!m_nodes.contains(srcId)) {
          m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
        }
        if (!m_nodes.contains(dstId)) {
          m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
        }
        assert(index == srcId * n + dstId);
        if (isAdj) {
          addEdge(index, std::make_pair(srcId, dstId));
        } else {
          addEdge(index, std::make_pair(srcId, dstId), val);
        }
        m_edges.at(index)->setMaxSpeed(defaultSpeed);
      }
    } else {
      // default case: read the file as a matrix with the first two elements being the number of rows and columns and
      // the following elements being the matrix elements
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Cannot find file: " + fileName));
      }
      Size rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        throw std::invalid_argument(Logger::buildExceptionMessage(
            "Adjacency matrix must be square. Rows: " + std::to_string(rows) +
            " Cols: " + std::to_string(cols)));
      }
      Size n{rows};
      if (n * n > std::numeric_limits<Id>::max()) {
        throw std::invalid_argument(Logger::buildExceptionMessage(
            "Matrix size is too large for the current type of Id."));
      }
      Id index{0};
      while (!file.eof()) {
        double value;
        file >> value;
        if (value < 0) {
          throw std::invalid_argument(Logger::buildExceptionMessage(
              "Adjacency matrix elements must be positive"));
        }
        if (value > 0) {
          const auto srcId{static_cast<Id>(index / n)};
          const auto dstId{static_cast<Id>(index % n)};
          if (!m_nodes.contains(srcId)) {
            m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
          }
          if (!m_nodes.contains(dstId)) {
            m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
          }
          assert(index == srcId * n + dstId);
          if (isAdj) {
            addEdge(index, std::make_pair(srcId, dstId));
          } else {
            addEdge(index, std::make_pair(srcId, dstId), value);
          }
          m_edges.at(index)->setMaxSpeed(defaultSpeed);
        }
        ++index;
      }
    }
  }

  void RoadNetwork::importCoordinates(const std::string& fileName) {
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (fileExt == "dsm") {
      // first input number is the number of nodes
      std::ifstream file{fileName};
      assert((void("Coordinates file not found."), file.is_open()));
      Size n;
      file >> n;
      if (n < m_nodes.size()) {
        throw std::invalid_argument(Logger::buildExceptionMessage(
            "Number of node cordinates in file is too small."));
      }
      double lat, lon;
      for (Size i{0}; i < n; ++i) {
        file >> lon >> lat;
        auto const& it{m_nodes.find(i)};
        if (it != m_nodes.cend()) {
          it->second->setCoords(std::make_pair(lon, lat));
        } else {
          Logger::warning(std::format("Node with id {} not found.", i));
        }
      }
    } else if (fileExt == "csv") {
      std::ifstream ifs{fileName};
      if (!ifs.is_open()) {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Cannot find file: " + fileName));
      }
      // Check if the first line is nodeId;lat;lon
      std::string line;
      std::getline(ifs, line);
      if (line != "id;lat;lon;type") {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Invalid file format."));
      }
      double dLat, dLon;
      while (!ifs.eof()) {
        std::getline(ifs, line);
        if (line.empty()) {
          continue;
        }
        std::istringstream iss{line};
        std::string nodeId, lat, lon, type;
        std::getline(iss, nodeId, ';');
        std::getline(iss, lat, ';');
        std::getline(iss, lon, ';');
        std::getline(iss, type, '\n');
        dLon = lon == "Nan" ? 0. : std::stod(lon);
        dLat = lat == "Nan" ? 0. : std::stod(lat);
        auto const& it{m_nodes.find(std::stoul(nodeId))};
        if (it != m_nodes.cend()) {
          it->second->setCoords(std::make_pair(dLat, dLon));
          if (type == "traffic_light" && !it->second->isTrafficLight()) {
            makeTrafficLight(it->first, 60);
          } else if (type == "roundabout" && !it->second->isRoundabout()) {
            makeRoundabout(it->first);
          }
        } else {
          Logger::warning(
              std::format("Node with id {} not found. Skipping coordinates ({}, {}).",
                          nodeId,
                          dLat,
                          dLon));
        }
      }
    } else {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("File extension not supported."));
    }
  }

  void RoadNetwork::importOSMNodes(const std::string& fileName) {
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (fileExt == "csv") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(
            Logger::buildExceptionMessage("Cannot find file: " + fileName));
      }
      std::string line;
      std::getline(file, line);  // skip first line
      Id nodeIndex{0};
      while (!file.eof()) {
        std::getline(file, line);
        if (line.empty()) {
          continue;
        }
        std::istringstream iss{line};
        std::string id, lat, lon, highway;
        // osmid;x;y;highway
        std::getline(iss, id, ';');
        std::getline(iss, lon, ';');
        std::getline(iss, lat, ';');
        std::getline(iss, highway, ';');
        if (highway.find("traffic_signals") != std::string::npos) {
          addNode<TrafficLight>(
              nodeIndex, 120, std::make_pair(std::stod(lat), std::stod(lon)));
        } else if (highway.find("roundabout") != std::string::npos) {
          addNode<Roundabout>(nodeIndex, std::make_pair(std::stod(lat), std::stod(lon)));
        } else {
          addNode<Intersection>(nodeIndex,
                                std::make_pair(std::stod(lat), std::stod(lon)));
          if ((highway.find("in_out") != std::string::npos) ||
              (highway.find("outgoing_only") != std::string::npos)) {
            Logger::debug(std::format("Setting node {} as an output node", nodeIndex));
            m_outputNodes.push_back(nodeIndex);
          }
          if ((highway.find("in_out") != std::string::npos) ||
              (highway.find("ingoing_only") != std::string::npos)) {
            Logger::debug(std::format("Setting node {} as an input node", nodeIndex));
            m_inputNodes.push_back(nodeIndex);
          }
        }
        m_nodeMapping.emplace(std::make_pair(id, nodeIndex));
        ++nodeIndex;
      }
    } else {
      Logger::error(std::format("File extension ({}) not supported", fileExt));
    }
    Logger::info(std::format("Successfully imported {} nodes", nNodes()));
  }

  void RoadNetwork::importOSMEdges(const std::string& fileName) {
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    auto const nNodes{m_nodes.size()};
    if (fileExt == "csv") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(Logger::buildExceptionMessage(
            std::format("File \'{}\' not found", fileName)));
      }
      std::string line;
      std::getline(file, line);  // skip first line
      while (!file.eof()) {
        std::getline(file, line);
        if (line.empty()) {
          continue;
        }
        std::istringstream iss{line};
        std::string sourceId, targetId, length, lanes, highway, maxspeed, name, geometry,
            coilcode;
        // u;v;length;highway;maxspeed;name;geometry;coilcode
        std::getline(iss, sourceId, ';');
        std::getline(iss, targetId, ';');
        std::getline(iss, length, ';');
        std::getline(iss, lanes, ';');
        std::getline(iss, highway, ';');
        std::getline(iss, maxspeed, ';');
        std::getline(iss, name, ';');
        std::getline(iss, geometry, ';');
        std::getline(iss, coilcode, '\n');
        if (maxspeed.empty()) {
          maxspeed = "30";  // Default to 30 km/h if no maxspeed is provided
        } else {
          try {
            std::stod(maxspeed);
          } catch (const std::invalid_argument& e) {
            maxspeed = "30";  // Default to 30 km/h if maxspeed is invalid
          }
        }

        if (lanes.empty()) {
          lanes = "1";  // Default to 1 lane if no value is provided
        } else {
          try {
            std::stoul(lanes);
          } catch (const std::invalid_argument& e) {
            lanes = "1";  // Default to 1 lane if lanes is invalid
          }
        }
        if (!m_nodeMapping.contains(sourceId)) {
          Logger::error(std::format("Node with id {} not found.", sourceId));
        }
        if (!m_nodeMapping.contains(targetId)) {
          Logger::error(std::format("Node with id {} not found.", targetId));
        }
        if (sourceId == targetId) {
          Logger::warning(
              std::format("Self loop detected: {}->{}. Skipping.", sourceId, targetId));
          continue;
        }

        auto const srcId{m_nodeMapping.at(sourceId)};
        auto const dstId{m_nodeMapping.at(targetId)};

        // Parse the geometry
        std::vector<std::pair<double, double>> coords;
        if (!geometry.empty()) {
          // Gemetri is LINESTRING(lon,lat lon,lat ...)
          std::istringstream geom{geometry};
          // Read until (
          std::string pair;
          std::getline(geom, pair, '(');
          while (std::getline(geom, pair, ',')) {
            pair.erase(pair.begin(),
                       std::find_if(pair.begin(), pair.end(), [](unsigned char ch) {
                         return !std::isspace(ch);
                       }));

            // Trim trailing spaces
            pair.erase(std::find_if(pair.rbegin(),
                                    pair.rend(),
                                    [](unsigned char ch) { return !std::isspace(ch); })
                           .base(),
                       pair.end());
            // Create a stream for each coordinate pair to split by comma
            std::istringstream pairStream(pair);
            std::string lon, lat;
            std::getline(pairStream, lon, ' ');
            std::getline(pairStream, lat);  // read the rest for latitude
            // Remove ')' from lat if present
            if (lat.back() == ')') {
              lat.pop_back();
            }
            // Note: The original code stored as (lat, lon) based on your comment.
            coords.emplace_back(std::stod(lon), std::stod(lat));
          }
        } else {
          coords.emplace_back(m_nodes.at(srcId)->coords().value());
          coords.emplace_back(m_nodes.at(dstId)->coords().value());
        }
        if (static_cast<unsigned long long>(srcId * nNodes + dstId) >
            std::numeric_limits<Id>::max()) {
          throw std::invalid_argument(Logger::buildExceptionMessage(
              std::format("Street id {}->{} would too large for the current type of Id.",
                          srcId,
                          dstId)));
        }
        Id streetId = srcId * nNodes + dstId;
        addEdge<Street>(streetId,
                        std::make_pair(srcId, dstId),
                        std::stod(length),
                        std::stod(maxspeed) / 3.6,
                        std::stoul(lanes),
                        name,
                        coords);
        if (!coilcode.empty()) {
          makeSpireStreet(streetId);
          auto& coil = edge<SpireStreet>(streetId);
          coil.setCode(static_cast<Id>(std::stoul(coilcode)));
        }
      }
    } else {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("File extension not supported"));
    }
    Logger::info(std::format("Successfully imported {} edges", nEdges()));
  }

  void RoadNetwork::exportNodes(std::string const& path) {
    // assert that path ends with ".csv"
    assert((void("Only csv export is supported."),
            path.substr(path.find_last_of(".")) == ".csv"));
    std::ofstream file{path};
    // Column names
    file << "id;lat;lon;type\n";
    for (auto const& [nodeId, pNode] : m_nodes) {
      file << nodeId << ';';
      if (pNode->coords().has_value()) {
        file << pNode->coords().value().first << ';' << pNode->coords().value().second;
      } else {
        file << "Nan;Nan";
      }
      if (pNode->isTrafficLight()) {
        file << ";traffic_light";
      } else if (pNode->isRoundabout()) {
        file << ";roundabout";
      } else {
        file << ";intersection";
      }
      file << '\n';
    }
    file.close();
  }
  void RoadNetwork::exportEdges(std::string const& path) {
    // assert that path ends with ".csv"
    assert((void("Only csv export is supported."),
            path.substr(path.find_last_of(".")) == ".csv"));
    std::ofstream file{path};
    // Column names
    file << "id;source_id;target_id;name;coil_code;geometry\n";
    for (auto const& [streetId, pStreet] : m_edges) {
      file << streetId << ';' << pStreet->source() << ';' << pStreet->target() << ';'
           << pStreet->name() << ';';
      if (pStreet->isSpire()) {
        file << dynamic_cast<SpireStreet&>(*pStreet).code() << ';';
      } else {
        file << ';';
      }
      if (!pStreet->geometry().empty()) {
        file << "LINESTRING(";
        for (size_t i{0}; i < pStreet->geometry().size(); ++i) {
          auto const& [lon, lat] = pStreet->geometry()[i];
          file << lon << ' ' << lat;
          if (i < pStreet->geometry().size() - 1) {
            file << ',';
          } else {
            file << ')';
          }
        }
      }
      file << '\n';
    }
    file.close();
  }
  void RoadNetwork::exportMatrix(std::string path, bool isAdj) {
    std::ofstream file{path};
    if (!file.is_open()) {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("Cannot open file: " + path));
    }
    auto const N{m_adjacencyMatrix.n()};
    file << N << '\t' << N;
    if (isAdj) {
      for (const auto& [source, target] : m_adjacencyMatrix.elements()) {
        file << '\n' << source * N + target << '\t' << 1;
      }
    } else {
      for (const auto& [id, street] : m_edges) {
        file << '\n' << id << '\t' << street->length();
      }
    }
  }

  TrafficLight& RoadNetwork::makeTrafficLight(Id const nodeId,
                                              Delay const cycleTime,
                                              Delay const counter) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<TrafficLight>(*pNode, cycleTime, counter);
    return node<TrafficLight>(nodeId);
  }

  Roundabout& RoadNetwork::makeRoundabout(Id nodeId) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Roundabout>(*pNode);
    return node<Roundabout>(nodeId);
  }

  Station& RoadNetwork::makeStation(Id nodeId, const unsigned int managementTime) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Station>(*pNode, managementTime);
    return node<Station>(nodeId);
  }
  void RoadNetwork::makeStochasticStreet(Id streetId, double const flowRate) {
    auto& pStreet = m_edges.at(streetId);
    pStreet = std::unique_ptr<StochasticStreet>(
        new StochasticStreet(std::move(*pStreet), flowRate));
  }
  void RoadNetwork::makeSpireStreet(Id streetId) {
    auto& pStreet = m_edges.at(streetId);
    if (pStreet->isStochastic()) {
      pStreet = std::unique_ptr<StochasticSpireStreet>(new StochasticSpireStreet(
          std::move(*pStreet), dynamic_cast<StochasticStreet&>(*pStreet).flowRate()));
      return;
    }
    pStreet = std::unique_ptr<SpireStreet>(new SpireStreet(std::move(*pStreet)));
  }

  void RoadNetwork::addStreet(Street&& street) { addEdge<Street>(std::move(street)); }

  const std::unique_ptr<Street>* RoadNetwork::street(Id source, Id destination) const {
    auto streetIt = std::find_if(m_edges.begin(),
                                 m_edges.end(),
                                 [source, destination](const auto& street) -> bool {
                                   return street.second->source() == source &&
                                          street.second->target() == destination;
                                 });
    if (streetIt == m_edges.end()) {
      return nullptr;
    }
    Size n = m_nodes.size();
    auto id1 = streetIt->first;
    auto id2 = source * n + destination;
    assert(id1 == id2);
    // if (id1 != id2) {
    //   std::cout << "node size: " << n << std::endl;
    //   std::cout << "Street id: " << id1 << std::endl;
    //   std::cout << "Nodes: " << id2 << std::endl;
    // }
    return &(streetIt->second);
  }

  const std::unique_ptr<Street>* RoadNetwork::oppositeStreet(Id streetId) const {
    if (!m_edges.contains(streetId)) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} does not exist: maybe it has changed "
                      "id once called buildAdj.",
                      streetId)));
    }
    const auto& nodePair = m_edges.at(streetId)->nodePair();
    return this->street(nodePair.second, nodePair.first);
  }
};  // namespace dsm
