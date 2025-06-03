
#include "../headers/RoadNetwork.hpp"

#include <algorithm>

namespace dsf {
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
    std::for_each(
        m_edges.cbegin(), m_edges.cend(), [this, &newStreetIds](auto const& pair) {
          auto const& pStreet{pair.second};
          auto const& forbiddenTurns{pStreet->forbiddenTurns()};
          if (forbiddenTurns.empty()) {
            return;
          }
          std::set<Id> newForbiddenTurns;
          for (auto const& streetId : forbiddenTurns) {
            newForbiddenTurns.insert(newStreetIds.at(streetId));
          }
          pStreet->setForbiddenTurns(newForbiddenTurns);
        });
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
        auto& trafficLight = static_cast<TrafficLight&>(*node);
        std::unordered_map<Id, std::unordered_map<Direction, TrafficLightCycle>> newCycles;
        for (auto const& [streetId, cycles] : trafficLight.cycles()) {
          newCycles.emplace(newStreetIds.at(streetId), std::move(cycles));
        }
        trafficLight.setCycles(newCycles);
      }
    }
  }

  void RoadNetwork::initTrafficLights(Delay const minGreenTime) {
    for (auto const& [id, pNode] : m_nodes) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = static_cast<TrafficLight&>(*pNode);
      if (!tl.streetPriorities().empty() || !tl.cycles().empty()) {
        continue;
      }
      auto const& inNeighbours = m_adjacencyMatrix.getCol(id);
      std::map<Id, int, std::greater<int>> capacities;
      std::unordered_map<Id, double> streetAngles;
      std::unordered_map<Id, double> maxSpeeds;
      std::unordered_map<Id, int> nLanes;
      std::unordered_map<Id, std::string> streetNames;
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
        // Logger::debug(std::format("Street {} with capacity {}", streetId, cap));
        capacities.emplace(streetId, cap);
        auto angle{pStreet->angle()};
        if (angle < 0.) {
          angle += 2 * std::numbers::pi;
        }
        streetAngles.emplace(streetId, angle);

        maxSpeeds.emplace(streetId, speed);
        nLanes.emplace(streetId, nLan);
        streetNames.emplace(streetId, pStreet->name());

        higherSpeed = std::max(higherSpeed, speed);
        lowerSpeed = std::min(lowerSpeed, speed);

        higherNLanes = std::max(higherNLanes, nLan);
        lowerNLanes = std::min(lowerNLanes, nLan);
      }
      {
        std::vector<std::pair<Id, double>> sortedAngles;
        std::copy(
            streetAngles.begin(), streetAngles.end(), std::back_inserter(sortedAngles));
        std::sort(sortedAngles.begin(),
                  sortedAngles.end(),
                  [](auto const& a, auto const& b) { return a.second < b.second; });
        streetAngles.clear();
        for (auto const& [streetId, angle] : sortedAngles) {
          streetAngles.emplace(streetId, angle);
        }
      }
      if (tl.streetPriorities().empty()) {
        /*************************************************************
         * 1. Check for street names with multiple occurrences
         * ***********************************************************/
        std::unordered_map<std::string, int> counts;
        for (auto const& [streetId, name] : streetNames) {
          if (name.empty()) {
            // Ignore empty names
            continue;
          }
          if (!counts.contains(name)) {
            counts[name] = 1;
          } else {
            ++counts.at(name);
          }
        }
        for (auto const& [name, count] : counts) {
          Logger::debug(std::format("Street name {} has {} occurrences", name, count));
        }
        for (auto const& [streetId, name] : streetNames) {
          if (!name.empty() && counts.at(name) > 1) {
            tl.addStreetPriority(streetId);
          }
        }
      }
      if (tl.streetPriorities().empty() && higherSpeed != lowerSpeed) {
        /*************************************************************
         * 2. Check for street names with same max speed
         * ***********************************************************/
        for (auto const& [sid, speed] : maxSpeeds) {
          if (speed == higherSpeed) {
            tl.addStreetPriority(sid);
          }
        }
      }
      if (tl.streetPriorities().empty() && higherNLanes != lowerNLanes) {
        /*************************************************************
         * 2. Check for street names with same number of lanes
         * ***********************************************************/
        for (auto const& [sid, nLan] : nLanes) {
          if (nLan == higherNLanes) {
            tl.addStreetPriority(sid);
          }
        }
      }
      if (tl.streetPriorities().empty()) {
        /*************************************************************
         * 3. Check for streets with opposite angles
         * ***********************************************************/
        auto const& streetId = streetAngles.begin()->first;
        auto const& angle = streetAngles.begin()->second;
        for (auto const& [streetId2, angle2] : streetAngles) {
          if (std::abs(angle - angle2) > 0.75 * std::numbers::pi) {
            tl.addStreetPriority(streetId);
            tl.addStreetPriority(streetId2);
            break;
          }
        }
      }
      if (tl.streetPriorities().empty()) {
        Logger::warning(
            std::format("Failed to auto-init Traffic Light {} - going random", id));
        // Assign first and third keys of capacity map
        auto it = capacities.begin();
        auto const& firstKey = it->first;
        ++it;
        ++it;
        auto const& thirdKey = it->first;
        tl.addStreetPriority(firstKey);
        tl.addStreetPriority(thirdKey);
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
      // if one of green times is less than 20, set it to 20 and refactor the other to have the sum to 120
      if (greenTimes.first < minGreenTime) {
        greenTimes.first = minGreenTime;
        greenTimes.second = tl.cycleTime() - minGreenTime;
      }
      if (greenTimes.second < minGreenTime) {
        greenTimes.second = minGreenTime;
        greenTimes.first = tl.cycleTime() - minGreenTime;
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
                        dsf::Direction::RIGHTANDSTRAIGHT,
                        TrafficLightCycle{static_cast<Delay>(greenTime * 2. / 3), phase});
            tl.setCycle(
                streetId,
                dsf::Direction::LEFT,
                TrafficLightCycle{
                    static_cast<Delay>(greenTime / 3.),
                    static_cast<Delay>(phase + static_cast<Delay>(greenTime * 2. / 3))});
            break;
          default:
            tl.setCycle(
                streetId, dsf::Direction::ANY, TrafficLightCycle{greenTime, phase});
            break;
        }
      });
    }
  }
  void RoadNetwork::autoMapStreetLanes() {
    std::for_each(m_nodes.cbegin(), m_nodes.cend(), [this](auto const& pair) {
      auto const& inNeighbours{m_adjacencyMatrix.getCol(pair.first)};
      auto const& outNeighbours{m_adjacencyMatrix.getRow(pair.first)};
      int maxPriority{0};
      std::for_each(
          inNeighbours.cbegin(),
          inNeighbours.cend(),
          [this, &pair, &maxPriority](auto const& inNodeId) {
            auto const& pStreet{m_edges.at(inNodeId * m_nodes.size() + pair.first)};
            maxPriority = std::max(maxPriority, pStreet->priority());
          });
      std::for_each(
          outNeighbours.cbegin(),
          outNeighbours.cend(),
          [this, &pair, &maxPriority](auto const& outNodeId) {
            auto const& pStreet{m_edges.at(pair.first * m_nodes.size() + outNodeId)};
            maxPriority = std::max(maxPriority, pStreet->priority());
          });
      std::for_each(
          inNeighbours.cbegin(),
          inNeighbours.cend(),
          [this, &pair, &outNeighbours, &maxPriority](auto const& inNodeId) {
            auto const& pInStreet{m_edges.at(inNodeId * m_nodes.size() + pair.first)};
            auto const nLanes{pInStreet->nLanes()};
            if (nLanes == 1) {
              return;
            }
            std::multiset<Direction> allowedTurns;
            std::for_each(
                outNeighbours.cbegin(),
                outNeighbours.cend(),
                [this, &pair, &pInStreet, &allowedTurns, &maxPriority](
                    auto const& outNodeId) {
                  auto const& pOutStreet{
                      m_edges.at(pair.first * m_nodes.size() + outNodeId)};
                  if (pOutStreet->target() == pInStreet->source() ||
                      pInStreet->forbiddenTurns().contains(pOutStreet->id())) {
                    return;
                  }
                  auto const deltaAngle{pOutStreet->deltaAngle(pInStreet->angle())};
                  auto const& outOppositeStreet{this->street(pair.first, outNodeId)};
                  if (!outOppositeStreet) {
                    return;
                  }
                  // Actually going straight means remain on the same road, thus...
                  if (((pInStreet->priority() == maxPriority) ==
                       (outOppositeStreet->get()->priority() == maxPriority)) &&
                      !allowedTurns.contains(Direction::STRAIGHT)) {
                    Logger::debug(
                        std::format("Street {} prioritized STRAIGHT", pInStreet->id()));
                    if (allowedTurns.contains(Direction::STRAIGHT) &&
                        !allowedTurns.contains(Direction::RIGHT)) {
                      allowedTurns.emplace(Direction::RIGHT);
                    } else {
                      allowedTurns.emplace(Direction::STRAIGHT);
                    }
                    // if (!allowedTurns.contains(Direction::STRAIGHT)) {
                    // allowedTurns.emplace(Direction::STRAIGHT);
                    // return;
                    // }
                  } else if (std::abs(deltaAngle) < std::numbers::pi) {
                    // Logger::debug(std::format("Angle in {} - angle out {}",
                    //                           pInStreet->angle(),
                    //                           pOutStreet->angle()));
                    // Logger::debug(std::format("Delta: {}", deltaAngle));
                    if (std::abs(deltaAngle) < std::numbers::pi / 8) {
                      Logger::debug(std::format("Street {} -> {} can turn STRAIGHT",
                                                pInStreet->source(),
                                                pInStreet->target()));
                      allowedTurns.emplace(Direction::STRAIGHT);
                    } else if (deltaAngle < 0.) {
                      Logger::debug(std::format("Street {} -> {} can turn RIGHT",
                                                pInStreet->source(),
                                                pInStreet->target()));
                      allowedTurns.emplace(Direction::RIGHT);
                    } else if (deltaAngle > 0.) {
                      Logger::debug(std::format("Street {} -> {} can turn LEFT",
                                                pInStreet->source(),
                                                pInStreet->target()));
                      allowedTurns.emplace(Direction::LEFT);
                    }
                  }
                });
            while (allowedTurns.size() < static_cast<size_t>(nLanes)) {
              if (allowedTurns.contains(Direction::STRAIGHT)) {
                allowedTurns.emplace(Direction::STRAIGHT);
              } else if (allowedTurns.contains(Direction::RIGHT)) {
                allowedTurns.emplace(Direction::RIGHT);
              } else if (allowedTurns.contains(Direction::LEFT)) {
                allowedTurns.emplace(Direction::LEFT);
              } else {
                allowedTurns.emplace(Direction::ANY);
              }
            }
            // If allowedTurns contains all RIGHT, STRAIGHT and LEFT, transform RIGHT into RIGHTANDSTRAIGHT
            if (allowedTurns.size() > static_cast<size_t>(nLanes)) {
              if (pair.second->isTrafficLight()) {
                auto& tl = static_cast<TrafficLight&>(*pair.second);
                auto const& cycles{tl.cycles()};
                if (cycles.contains(pInStreet->id())) {
                  if (cycles.size() == static_cast<size_t>(nLanes)) {
                    // Replace with the traffic light cycles
                    Logger::debug(
                        std::format("Using traffic light {} cycles for street {} -> {}",
                                    tl.id(),
                                    pInStreet->source(),
                                    pInStreet->target()));
                    auto const& cycle{cycles.at(pInStreet->id())};
                    allowedTurns.clear();
                    for (auto const& [direction, cycle] : cycle) {
                      allowedTurns.emplace(direction);
                    }
                  } else if (cycles.at(pInStreet->id())
                                 .contains(Direction::LEFTANDSTRAIGHT)) {
                    allowedTurns.erase(Direction::LEFT);
                    allowedTurns.erase(Direction::STRAIGHT);
                    allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                  } else if (cycles.at(pInStreet->id())
                                 .contains(Direction::RIGHTANDSTRAIGHT)) {
                    allowedTurns.erase(Direction::RIGHT);
                    allowedTurns.erase(Direction::STRAIGHT);
                    allowedTurns.emplace(Direction::RIGHTANDSTRAIGHT);
                  }
                }
              }
            }
            if (allowedTurns.size() > static_cast<size_t>(nLanes)) {
              // if one is duplicate, remove it
              std::set<Direction> uniqueDirections;
              std::copy(allowedTurns.begin(),
                        allowedTurns.end(),
                        std::inserter(uniqueDirections, uniqueDirections.begin()));
              allowedTurns.clear();
              std::copy(uniqueDirections.begin(),
                        uniqueDirections.end(),
                        std::inserter(allowedTurns, allowedTurns.begin()));
            }
            while (allowedTurns.size() < static_cast<size_t>(nLanes)) {
              if (allowedTurns.contains(Direction::STRAIGHT)) {
                allowedTurns.emplace(Direction::STRAIGHT);
              } else if (allowedTurns.contains(Direction::RIGHT)) {
                allowedTurns.emplace(Direction::RIGHT);
              } else if (allowedTurns.contains(Direction::LEFT)) {
                allowedTurns.emplace(Direction::LEFT);
              } else {
                allowedTurns.emplace(Direction::ANY);
              }
            }
            switch (nLanes) {
              case 1:
                // Leaving Direction::ANY for one lane streets is the less painful option
                break;
              case 2:
                if (allowedTurns.contains(Direction::STRAIGHT) &&
                    allowedTurns.contains(Direction::RIGHT) &&
                    allowedTurns.contains(Direction::LEFT)) {
                  if (pair.second->isTrafficLight()) {
                    auto& tl = static_cast<TrafficLight&>(*pair.second);
                    auto const& cycles{tl.cycles()};
                    if (cycles.contains(pInStreet->id())) {
                      auto const& cycle{cycles.at(pInStreet->id())};
                      if (cycle.contains(Direction::LEFTANDSTRAIGHT) &&
                          cycle.contains(Direction::RIGHT)) {
                        allowedTurns.erase(Direction::LEFT);
                        allowedTurns.erase(Direction::STRAIGHT);
                        allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                        break;
                      }
                    }
                  }
                  allowedTurns.clear();
                  allowedTurns.emplace(Direction::RIGHTANDSTRAIGHT);
                  allowedTurns.emplace(Direction::LEFT);
                }
                if (allowedTurns.size() > 2) {
                  // Remove duplicates
                  std::set<Direction> uniqueDirections;
                  std::copy(allowedTurns.begin(),
                            allowedTurns.end(),
                            std::inserter(uniqueDirections, uniqueDirections.begin()));
                  allowedTurns.clear();
                  std::copy(uniqueDirections.begin(),
                            uniqueDirections.end(),
                            std::inserter(allowedTurns, allowedTurns.begin()));
                }
                [[fallthrough]];
              default:
                // Logger::info(std::format(
                //     "Street {}->{} with {} lanes and {} allowed turns",
                //     pInStreet->source(),
                //     pInStreet->target(),
                //     nLanes,
                //     allowedTurns.size()));
                assert(allowedTurns.size() == static_cast<size_t>(nLanes));
                // Logger::info(
                //     std::format("Street {}->{} with {} lanes and {} allowed turns",
                //                 pInStreet->source(),
                //                 pInStreet->target(),
                //                 nLanes,
                //                 allowedTurns.size()));
                std::vector<Direction> newMapping(nLanes);
                auto it{allowedTurns.cbegin()};
                for (size_t i{0}; i < allowedTurns.size(); ++i, ++it) {
                  newMapping[i] = *it;
                }
                // If the last one is RIGHTANDSTRAIGHT, move it in front
                if (newMapping.back() == Direction::RIGHTANDSTRAIGHT) {
                  std::rotate(
                      newMapping.rbegin(), newMapping.rbegin() + 1, newMapping.rend());
                }
                pInStreet->setLaneMapping(newMapping);
            }
          });
    });
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
    if (fileExt == "dsf") {
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
    if (fileExt == "dsf") {
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
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::invalid_argument(
          Logger::buildExceptionMessage(std::format("File \'{}\' not found", fileName)));
    }
    std::unordered_map<Id, std::string> mapForbiddenTurns;
    std::string line;
    std::getline(file, line);  // skip first line
    while (!file.eof()) {
      std::getline(file, line);
      if (line.empty()) {
        continue;
      }
      std::istringstream iss{line};
      std::string sourceId, targetId, length, lanes, highway, maxspeed, name, geometry,
          forbiddenTurns, coilcode;
      // sourceId;targetId;length;highway;maxspeed;name;geometry;forbiddenTurns;coilcode
      std::getline(iss, sourceId, ';');
      std::getline(iss, targetId, ';');
      std::getline(iss, length, ';');
      std::getline(iss, lanes, ';');
      std::getline(iss, highway, ';');
      std::getline(iss, maxspeed, ';');
      std::getline(iss, name, ';');
      std::getline(iss, geometry, ';');
      std::getline(iss, forbiddenTurns, ';');
      std::getline(iss, coilcode, '\n');
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
      auto dLength{0.};
      try {
        dLength = std::stod(length);
      } catch (const std::invalid_argument& e) {
        Logger::error(
            std::format("Invalid length {} for edge {}->{}", length, srcId, dstId));
        continue;
      }
      auto dMaxSpeed{0.};
      try {
        dMaxSpeed = std::stod(maxspeed);
      } catch (const std::invalid_argument& e) {
        dMaxSpeed = 30.;  // Default to 30 km/h if maxspeed is invalid
      }
      int iLanes{0};
      try {
        iLanes = std::stoi(lanes);
      } catch (const std::invalid_argument& e) {
        iLanes = 1;  // Default to 1 lane if lanes is invalid
      }

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
          auto dLon{0.}, dLat{0.};
          try {
            dLon = std::stod(lon);
            dLat = std::stod(lat);
          } catch (const std::invalid_argument& e) {
            Logger::error(std::format(
                "Invalid coordinates ({}, {}) for edge {}->{}", lon, lat, srcId, dstId));
          }
          // Note: The original code stored as (lat, lon) based on your comment.
          coords.emplace_back(dLon, dLat);
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
                      dLength,
                      dMaxSpeed / 3.6,
                      iLanes,
                      name,
                      coords);
      if (!coilcode.empty()) {
        makeSpireStreet(streetId);
        auto& coil = edge<SpireStreet>(streetId);
        try {
          auto const coilId{static_cast<Id>(std::stoul(coilcode))};
          coil.setCode(coilId);
        } catch (const std::invalid_argument& e) {
          Logger::warning(std::format(
              "Invalid coil code {} for edge {}->{}", coilcode, srcId, dstId));
        }
      }
      // if (!forbiddenTurns.empty()) {
      //   mapForbiddenTurns.emplace(streetId, forbiddenTurns);
      // }
    }
    // Parse forbidden turns
    for (auto const& [streetId, forbiddenTurns] : mapForbiddenTurns) {
      auto const& pStreet{m_edges.at(streetId)};
      std::istringstream iss{forbiddenTurns};
      std::string pair;
      while (std::getline(iss, pair, ',')) {
        // Decompose pair = sourceId-targetId
        std::istringstream pairStream(pair);
        std::string strSourceId, strTargetId;
        std::getline(pairStream, strSourceId, '-');
        // targetId is the remaining part
        std::getline(pairStream, strTargetId);

        Id sourceId{0}, targetId{0};
        try {
          sourceId = m_nodeMapping.at(strSourceId);
          targetId = m_nodeMapping.at(strTargetId);
        } catch (const std::out_of_range& e) {
          Logger::warning(
              std::format("Invalid forbidden turn {}->{} for street {}. Skipping.",
                          strSourceId,
                          strTargetId,
                          streetId));
          continue;
        }

        auto const forbiddenStreetId{sourceId * nNodes + targetId};
        pStreet->addForbiddenTurn(forbiddenStreetId);
      }
    }

    Logger::info(std::format("Successfully imported {} edges", nEdges()));
  }
  void RoadNetwork::importTrafficLights(const std::string& fileName) {
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::invalid_argument(
          Logger::buildExceptionMessage("Cannot find file: " + fileName));
    }
    std::unordered_map<Id, Delay> storedGreenTimes;
    std::string line;
    std::getline(file, line);  // skip first line
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      std::istringstream iss{line};
      std::string strId, streetSource, strCycleTime, strGT;
      // id;streetSource;cycleTime;greenTime
      std::getline(iss, strId, ';');
      std::getline(iss, streetSource, ';');
      std::getline(iss, strCycleTime, ';');
      std::getline(iss, strGT, '\n');

      auto const cycleTime{static_cast<Delay>(std::stoul(strCycleTime))};
      // Cast node(id) to traffic light
      auto& pNode{m_nodes.at(m_nodeMapping.at(strId))};
      if (!pNode->isTrafficLight()) {
        pNode = std::make_unique<TrafficLight>(
            pNode->id(), cycleTime, pNode->coords().value());
      }
      auto& tl = static_cast<TrafficLight&>(*pNode);
      auto const srcId{m_nodeMapping.at(streetSource)};
      auto const streetId{srcId * m_nodes.size() + pNode->id()};
      auto const greenTime{static_cast<Delay>(std::stoul(strGT))};
      if (!storedGreenTimes.contains(pNode->id())) {
        storedGreenTimes.emplace(pNode->id(), greenTime);
      }
      auto const storedGT{storedGreenTimes.at(pNode->id())};
      if (storedGT == greenTime) {
        auto cycle = TrafficLightCycle(greenTime, 0);
        tl.setCycle(streetId, dsf::Direction::ANY, cycle);
      } else {
        auto cycle = TrafficLightCycle(greenTime, storedGT);
        tl.setCycle(streetId, dsf::Direction::ANY, cycle);
      }
    }
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
      } else if (std::find(m_inputNodes.begin(), m_inputNodes.end(), nodeId) !=
                     m_inputNodes.end() ||
                 std::find(m_outputNodes.begin(), m_outputNodes.end(), nodeId) !=
                     m_outputNodes.end()) {
        file << ";io";
      } else {
        file << ";";
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
    file << "id;source_id;target_id;length;capacity;name;coil_code;geometry\n";
    for (auto const& [streetId, pStreet] : m_edges) {
      file << streetId << ';' << pStreet->source() << ';' << pStreet->target() << ';'
           << pStreet->length() << ';' << pStreet->capacity() << ';' << pStreet->name()
           << ';';
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
};  // namespace dsf
