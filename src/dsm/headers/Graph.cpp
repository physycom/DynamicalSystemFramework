
#include "Graph.hpp"

namespace dsm {
  Graph::Graph()
      : m_adjacency{SparseMatrix<bool>()},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {}

  Graph::Graph(const SparseMatrix<bool>& adj)
      : m_adjacency{adj},
        m_maxAgentCapacity{std::numeric_limits<unsigned long long>::max()} {
    assert(adj.getRowDim() == adj.getColDim());
    auto n{static_cast<Size>(adj.getRowDim())};
    for (const auto& [id, value] : adj) {
      const auto srcId{static_cast<Id>(id / n)};
      const auto dstId{static_cast<Id>(id % n)};
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
      : m_adjacency{SparseMatrix<bool>()} {
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
    const auto oldStreetSet{std::move(m_streets)};
    m_streets.clear();
    const auto n{static_cast<Size>(m_nodes.size())};
    std::unordered_map<Id, Id> newStreetIds;
    for (const auto& [streetId, street] : oldStreetSet) {
      const auto srcId{street->u()};
      const auto dstId{street->v()};
      const auto newStreetId{static_cast<Id>(srcId * n + dstId)};
      if (m_streets.contains(newStreetId)) {
        throw std::invalid_argument(
            buildLog(std::format("Street with same id ({}) from {} to {} already exists.",
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

  void Graph::m_setStreetAngles() {
    for (const auto& [streetId, street] : m_streets) {
      const auto& srcNode{m_nodes.at(street->u())};
      const auto& dstNode{m_nodes.at(street->v())};
      if (srcNode->coords().has_value() && dstNode->coords().has_value()) {
        street->setAngle(srcNode->coords().value(), dstNode->coords().value());
      }
    }
  }

  void Graph::buildAdj() {
    // find max values in streets node pairs
    m_maxAgentCapacity = 0;
    const auto maxNode{static_cast<Id>(m_nodes.size())};
    m_adjacency.reshape(maxNode, maxNode);
    for (const auto& [streetId, street] : m_streets) {
      m_maxAgentCapacity += street->capacity();
      m_adjacency.insert(street->nodePair().first, street->nodePair().second, true);
    }
    this->m_reassignIds();
    this->m_setStreetAngles();
  }

  void Graph::buildStreetAngles() {
    for (auto const& street : m_streets) {
      const auto& node1{m_nodes.at(street.second->u())};
      const auto& node2{m_nodes.at(street.second->v())};
      street.second->setAngle(node1->coords().value(), node2->coords().value());
    }
  }

  void Graph::adjustNodeCapacities() {
    int16_t value;
    for (Id nodeId = 0; nodeId < m_nodes.size(); ++nodeId) {
      value = 0;
      for (const auto& [streetId, _] : m_adjacency.getCol(nodeId, true)) {
        auto const& pStreet{m_streets.at(streetId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      auto const& pNode{m_nodes.at(nodeId)};
      pNode->setCapacity(value);
      value = 0;
      for (const auto& [streetId, _] : m_adjacency.getRow(nodeId, true)) {
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
        throw std::invalid_argument(buildLog("Cannot find file: " + fileName));
      }
      Size rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        throw std::invalid_argument(buildLog("Adjacency matrix must be square"));
      }
      Size n{rows};
      m_adjacency = SparseMatrix<bool>(n, n);
      // each line has 2 elements
      while (!file.eof()) {
        Id index;
        double val;
        file >> index >> val;
        m_adjacency.insert(index, val);
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
        throw std::invalid_argument(buildLog("Cannot find file: " + fileName));
      }
      Size rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        throw std::invalid_argument(
            buildLog("Adjacency matrix must be square. Rows: " + std::to_string(rows) +
                     " Cols: " + std::to_string(cols)));
      }
      Size n{rows};
      if (n * n > std::numeric_limits<Id>::max()) {
        throw std::invalid_argument(
            buildLog("Matrix size is too large for the current type of Id."));
      }
      m_adjacency = SparseMatrix<bool>(n, n);
      Id index{0};
      while (!file.eof()) {
        double value;
        file >> value;
        if (value < 0) {
          throw std::invalid_argument(
              buildLog("Adjacency matrix elements must be positive"));
        }
        if (value > 0) {
          m_adjacency.insert(index, true);
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
        throw std::invalid_argument(
            buildLog("Number of node cordinates in file is too small."));
      }
      double lat, lon;
      for (Size i{0}; i < n; ++i) {
        file >> lat >> lon;
        auto const& it{m_nodes.find(i)};
        if (it != m_nodes.cend()) {
          it->second->setCoords(std::make_pair(lat, lon));
        } else {
          std::cerr << std::format(
                           "\033[38;2;130;30;180mWARNING ({}:{}): Node with id {} not "
                           "found.\033[0m",
                           __FILE__,
                           __LINE__,
                           i)
                    << std::endl;
        }
      }
    } else if (fileExt == "csv") {
      std::ifstream ifs{fileName};
      if (!ifs.is_open()) {
        throw std::invalid_argument(buildLog("Cannot find file: " + fileName));
      }
      // Check if the first line is nodeId;lat;lon
      std::string line;
      std::getline(ifs, line);
      if (line != "nodeId;lat;lon") {
        throw std::invalid_argument(buildLog("Invalid file format."));
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
      throw std::invalid_argument(buildLog("File extension not supported."));
    }
  }

  void Graph::importOSMNodes(const std::string& fileName) {
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (fileExt == "csv") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(buildLog("Cannot find file: " + fileName));
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
        auto const nodeId{static_cast<Id>(std::stoul(id))};
        if (highway.find("traffic_signals") != std::string::npos) {
          addNode<TrafficLight>(
              nodeIndex, 60, std::make_pair(std::stod(lat), std::stod(lon)));
        } else if (highway.find("roundabout") != std::string::npos) {
          addNode<Roundabout>(nodeIndex, std::make_pair(std::stod(lat), std::stod(lon)));
        } else {
          addNode<Intersection>(nodeIndex,
                                std::make_pair(std::stod(lat), std::stod(lon)));
        }
        m_nodeMapping.emplace(std::make_pair(nodeId, nodeIndex));
        ++nodeIndex;
      }
    } else {
      throw std::invalid_argument(buildLog("File extension not supported"));
    }
  }

  void Graph::importOSMEdges(const std::string& fileName) {
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    auto const nNodes{m_nodes.size()};
    if (fileExt == "csv") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        throw std::invalid_argument(
            buildLog(std::format("File \'{}\' not found", fileName)));
      }
      std::string line;
      std::getline(file, line);  // skip first line
      while (!file.eof()) {
        std::getline(file, line);
        if (line.empty()) {
          continue;
        }
        std::istringstream iss{line};
        std::string sourceId, targetId, length, oneway, lanes, highway, maxspeed, name;
        // u;v;length;oneway;highway;maxspeed;name
        std::getline(iss, sourceId, ';');
        std::getline(iss, targetId, ';');
        std::getline(iss, length, ';');
        std::getline(iss, oneway, ';');
        std::getline(iss, lanes, ';');
        std::getline(iss, highway, ';');
        std::getline(iss, maxspeed, ';');
        std::getline(iss, name, ';');
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
        if (!m_nodeMapping.contains(std::stoul(sourceId))) {
          std::cerr << std::format(
                           "\033[38;5;196mERROR ({}:{}): Node with id {} not "
                           "found.\033[0m",
                           __FILE__,
                           __LINE__,
                           sourceId)
                    << std::endl;
          std::abort();
        }
        if (!m_nodeMapping.contains(std::stoul(targetId))) {
          std::cerr << std::format(
                           "\033[38;5;196mERROR ({}:{}): Node with id {} not "
                           "found.\033[0m",
                           __FILE__,
                           __LINE__,
                           targetId)
                    << std::endl;
          std::abort();
        }
        auto const srcId{m_nodeMapping.at(std::stoul(sourceId))};
        auto const dstId{m_nodeMapping.at(std::stoul(targetId))};
        Id streetId = srcId * nNodes + dstId;
        addEdge<Street>(streetId,
                        std::make_pair(srcId, dstId),
                        std::stod(length),
                        std::stod(maxspeed),
                        std::stoul(lanes),
                        name);
      }
    } else {
      throw std::invalid_argument(buildLog("File extension not supported"));
    }
  }

  void Graph::exportMatrix(std::string path, bool isAdj) {
    std::ofstream file{path};
    if (!file.is_open()) {
      throw std::invalid_argument(buildLog("Cannot open file: " + path));
    }
    if (isAdj) {
      file << m_adjacency.getRowDim() << '\t' << m_adjacency.getColDim();
      for (const auto& [id, value] : m_adjacency) {
        file << '\n' << id << '\t' << value;
      }
    } else {
      file << m_adjacency.getRowDim() << '\t' << m_adjacency.getColDim();
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
      throw std::invalid_argument(
          buildLog(std::format("Street with id {} from {} to {} already exists.",
                               street->id(),
                               street->nodePair().first,
                               street->nodePair().second)));
    }
    // emplace nodes
    const auto srcId{street->nodePair().first};
    const auto dstId{street->nodePair().second};
    if (!m_nodes.contains(srcId)) {
      m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
    }
    if (!m_nodes.contains(dstId)) {
      m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
    }
    // emplace street
    m_streets.emplace(std::make_pair(street->id(), std::move(street)));
  }

  void Graph::addStreet(const Street& street) {
    if (m_streets.contains(street.id())) {
      throw std::invalid_argument(
          buildLog(std::format("Street with id {} from {} to {} already exists.",
                               street.id(),
                               street.nodePair().first,
                               street.nodePair().second)));
    }
    // emplace nodes
    const auto srcId{street.nodePair().first};
    const auto dstId{street.nodePair().second};
    if (!m_nodes.contains(srcId)) {
      m_nodes.emplace(srcId, std::make_unique<Intersection>(srcId));
    }
    if (!m_nodes.contains(dstId)) {
      m_nodes.emplace(dstId, std::make_unique<Intersection>(dstId));
    }
    // emplace street
    m_streets.emplace(std::make_pair(street.id(), std::make_unique<Street>(street)));
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
      throw std::invalid_argument(
          buildLog(std::format("Street with id {} does not exist: maybe it has changed "
                               "id once called buildAdj.",
                               streetId)));
    }
    const auto& nodePair = m_streets.at(streetId)->nodePair();
    return this->street(nodePair.second, nodePair.first);
  }
};  // namespace dsm
