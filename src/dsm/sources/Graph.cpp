
#include "../headers/Graph.hpp"

#include <algorithm>

namespace dsm {
  Graph::Graph()
      : m_adjacencyMatrix{AdjacencyMatrix()},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {}

  Graph::Graph(AdjacencyMatrix const& adj)
      : m_adjacencyMatrix{adj},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {
    auto n{static_cast<Size>(adj.n())};
    for (const auto& [srcId, dstId] : adj.elements()) {
      auto const id{srcId * n + dstId};
      if (!m_nodes.contains(srcId)) {
        m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
      }
      if (!m_nodes.contains(dstId)) {
        m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
      }
      m_streets.emplace(id, std::make_unique<Street>(id, std::make_pair(srcId, dstId)));
    }
  }

  Graph::Graph(const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet)
      : m_adjacencyMatrix{AdjacencyMatrix(streetSet)} {
    for (auto& street : streetSet) {
      m_streets.emplace(street.second->id(), street.second.get());

      Id node1 = street.second->nodePair().first;
      Id node2 = street.second->nodePair().second;
      m_nodes.emplace(node1, std::make_unique<Intersection>(node1));
      m_nodes.emplace(node2, std::make_unique<Intersection>(node2));
    }

    buildAdj();
  }

  void Graph::m_reassignIds() {
    // not sure about this, might need a bit more work
    auto const oldStreetSet{std::move(m_streets)};
    auto const N{nNodes()};
    std::unordered_map<Id, Id> newStreetIds;
    for (const auto& [streetId, street] : oldStreetSet) {
      const auto srcId{street->source()};
      const auto dstId{street->target()};
      const auto newStreetId{static_cast<Id>(srcId * N + dstId)};
      if (m_streets.contains(newStreetId)) {
        throw std::invalid_argument(Logger::buildExceptionMessage(
            std::format("Street with same id ({}) from {} to {} already exists.",
                        newStreetId,
                        srcId,
                        dstId)));
      }
      if (street->isSpire() && street->isStochastic()) {
        m_streets.emplace(newStreetId,
                          std::make_unique<StochasticSpireStreet>(
                              newStreetId,
                              *street,
                              dynamic_cast<StochasticSpireStreet&>(*street).flowRate()));
      } else if (street->isStochastic()) {
        m_streets.emplace(newStreetId,
                          std::make_unique<StochasticStreet>(
                              newStreetId,
                              *street,
                              dynamic_cast<StochasticStreet&>(*street).flowRate()));
      } else if (street->isSpire()) {
        m_streets.emplace(newStreetId,
                          std::make_unique<SpireStreet>(newStreetId, *street));
      } else {
        m_streets.emplace(newStreetId, std::make_unique<Street>(newStreetId, *street));
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

  void Graph::buildAdj() {
    // find max values in streets node pairs
    m_maxAgentCapacity = 0;
    for (const auto& [streetId, pStreet] : m_streets) {
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

  void Graph::adjustNodeCapacities() {
    int16_t value;
    for (Id nodeId = 0; nodeId < m_nodes.size(); ++nodeId) {
      value = 0;
      auto const N{nNodes()};
      for (const auto& sourceId : m_adjacencyMatrix.getCol(nodeId)) {
        auto const streetId{sourceId * N + nodeId};
        auto const& pStreet{m_streets.at(streetId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      auto const& pNode{m_nodes.at(nodeId)};
      pNode->setCapacity(value);
      value = 0;
      for (const auto& targetId : m_adjacencyMatrix.getRow(nodeId)) {
        auto const streetId{nodeId * N + targetId};
        auto const& pStreet{m_streets.at(streetId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      pNode->setTransportCapacity(value == 0 ? 1 : value);
      if (pNode->capacity() == 0) {
        pNode->setCapacity(value);
      }
    }
  }

  void Graph::importMatrix(const std::string& fileName, bool isAdj, double defaultSpeed) {
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
          addEdge<Street>(index, std::make_pair(srcId, dstId));
        } else {
          addEdge<Street>(index, std::make_pair(srcId, dstId), val);
        }
        m_streets.at(index)->setMaxSpeed(defaultSpeed);
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
            addEdge<Street>(index, std::make_pair(srcId, dstId));
          } else {
            addEdge<Street>(index, std::make_pair(srcId, dstId), value);
          }
          m_streets.at(index)->setMaxSpeed(defaultSpeed);
        }
        ++index;
      }
    }
  }

  void Graph::importCoordinates(const std::string& fileName) {
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
        file >> lat >> lon;
        auto const& it{m_nodes.find(i)};
        if (it != m_nodes.cend()) {
          it->second->setCoords(std::make_pair(lat, lon));
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
      if (line != "nodeId;lat;lon") {
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
        std::string nodeId, lat, lon;
        std::getline(iss, nodeId, ';');
        std::getline(iss, lat, ';');
        std::getline(iss, lon, '\n');
        dLat = lat == "Nan" ? 0. : std::stod(lat);
        dLon = lon == "Nan" ? 0. : std::stod(lon);
        auto const& it{m_nodes.find(std::stoul(nodeId))};
        if (it != m_nodes.cend()) {
          it->second->setCoords(std::make_pair(dLat, dLon));
        } else {
          std::cerr << std::format(
                           "\033[38;2;130;30;180mWARNING ({}:{}): Node with id {} not "
                           "found.\033[0m",
                           __FILE__,
                           __LINE__,
                           nodeId)
                    << std::endl;
        }
      }
    } else {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("File extension not supported."));
    }
  }

  void Graph::importOSMNodes(const std::string& fileName) {
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
              nodeIndex, 60, std::make_pair(std::stod(lat), std::stod(lon)));
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
              (highway.find("incoming_only") != std::string::npos)) {
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

  void Graph::importOSMEdges(const std::string& fileName) {
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
        std::string sourceId, targetId, length, lanes, highway, maxspeed, name, geometry;
        // u;v;length;highway;maxspeed;name;geometry
        std::getline(iss, sourceId, ';');
        std::getline(iss, targetId, ';');
        std::getline(iss, length, ';');
        std::getline(iss, lanes, ';');
        std::getline(iss, highway, ';');
        std::getline(iss, maxspeed, ';');
        std::getline(iss, name, ';');
        std::getline(iss, geometry, '\n');
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
      }
    } else {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("File extension not supported"));
    }
    Logger::info(std::format("Successfully imported {} edges", nEdges()));
  }

  void Graph::exportOSMNodes(std::string const& path) {
    // assert that path ends with ".csv"
    assert((void("Only csv export is supported."),
            path.substr(path.find_last_of(".")) == ".csv"));
    std::ofstream file{path};
    // Column names
    file << "id;x;y\n";
    for (auto const& [nodeId, pNode] : m_nodes) {
      file << nodeId << ';' << pNode->coords().value().second << ';'
           << pNode->coords().value().first << '\n';
    }
    file.close();
  }
  void Graph::exportOSMEdges(std::string const& path) {
    // assert that path ends with ".csv"
    assert((void("Only csv export is supported."),
            path.substr(path.find_last_of(".")) == ".csv"));
    std::ofstream file{path};
    // Column names
    file << "id;source;target;name;geometry\n";
    for (auto const& [streetId, pStreet] : m_streets) {
      file << streetId << ';' << pStreet->source() << ';' << pStreet->target() << ';'
           << pStreet->name() << ';';
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
  void Graph::exportMatrix(std::string path, bool isAdj) {
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
      for (const auto& [id, street] : m_streets) {
        file << '\n' << id << '\t' << street->length();
      }
    }
  }

  void Graph::exportCoordinates(std::string const& path) {
    // assert that path ends with ".csv"
    assert((void("Only csv export is supported."),
            path.substr(path.find_last_of(".")) == ".csv"));
    std::ofstream file{path};
    // Column names
    file << "nodeId;lat;lon\n";
    for (const auto& [id, node] : m_nodes) {
      file << id << ';';
      if (node->coords().has_value()) {
        file << node->coords().value().first << ';' << node->coords().value().second;
      } else {
        file << "Nan;Nan";
      }
      file << '\n';
    }
    file.close();
  }

  void Graph::addNode(std::unique_ptr<Node> node) {
    m_nodes.emplace(std::make_pair(node->id(), std::move(node)));
  }

  TrafficLight& Graph::makeTrafficLight(Id const nodeId,
                                        Delay const cycleTime,
                                        Delay const counter) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<TrafficLight>(*pNode, cycleTime, counter);
    return dynamic_cast<TrafficLight&>(*pNode);
  }

  Roundabout& Graph::makeRoundabout(Id nodeId) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Roundabout>(*pNode);
    return dynamic_cast<Roundabout&>(*pNode);
  }

  Station& Graph::makeStation(Id nodeId, const unsigned int managementTime) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Station>(*pNode, managementTime);
    return dynamic_cast<Station&>(*pNode);
  }
  StochasticStreet& Graph::makeStochasticStreet(Id streetId, double const flowRate) {
    auto& pStreet = m_streets.at(streetId);
    pStreet = std::make_unique<StochasticStreet>(pStreet->id(), *pStreet, flowRate);
    return dynamic_cast<StochasticStreet&>(*pStreet);
  }
  void Graph::makeSpireStreet(Id streetId) {
    auto& pStreet = m_streets.at(streetId);
    if (pStreet->isStochastic()) {
      pStreet = std::make_unique<StochasticSpireStreet>(
          pStreet->id(), *pStreet, dynamic_cast<StochasticStreet&>(*pStreet).flowRate());
      return;
    }
    pStreet = std::make_unique<SpireStreet>(pStreet->id(), *pStreet);
  }

  void Graph::addStreet(std::unique_ptr<Street> street) {
    if (m_streets.contains(street->id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} from {} to {} already exists.",
                      street->id(),
                      street->source(),
                      street->target())));
    }
    // emplace nodes
    const auto srcId{street->source()};
    const auto dstId{street->target()};
    if (!m_nodes.contains(srcId)) {
      m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
    }
    if (!m_nodes.contains(dstId)) {
      m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
    }
    // emplace street
    m_streets.emplace(std::make_pair(street->id(), std::move(street)));
    m_adjacencyMatrix.insert(srcId, dstId);
  }

  void Graph::addStreet(const Street& street) {
    if (m_streets.contains(street.id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} from {} to {} already exists.",
                      street.id(),
                      street.source(),
                      street.target())));
    }
    // emplace nodes
    const auto srcId{street.source()};
    const auto dstId{street.target()};
    if (!m_nodes.contains(srcId)) {
      m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
    }
    if (!m_nodes.contains(dstId)) {
      m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
    }
    // emplace street
    m_streets.emplace(std::make_pair(street.id(), std::make_unique<Street>(street)));
    m_adjacencyMatrix.insert(srcId, dstId);
  }

  const std::unique_ptr<Street>* Graph::street(Id source, Id destination) const {
    auto streetIt = std::find_if(m_streets.begin(),
                                 m_streets.end(),
                                 [source, destination](const auto& street) -> bool {
                                   return street.second->nodePair().first == source &&
                                          street.second->nodePair().second == destination;
                                 });
    if (streetIt == m_streets.end()) {
      return nullptr;
    }
    Size n = m_nodes.size();
    auto id1 = streetIt->first;
    auto id2 = source * n + destination;
    assert(id1 == id2);
    if (id1 != id2) {
      std::cout << "node size: " << n << std::endl;
      std::cout << "Street id: " << id1 << std::endl;
      std::cout << "Nodes: " << id2 << std::endl;
    }
    return &(streetIt->second);
  }

  const std::unique_ptr<Street>* Graph::oppositeStreet(Id streetId) const {
    if (!m_streets.contains(streetId)) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} does not exist: maybe it has changed "
                      "id once called buildAdj.",
                      streetId)));
    }
    const auto& nodePair = m_streets.at(streetId)->nodePair();
    return this->street(nodePair.second, nodePair.first);
  }
};  // namespace dsm
