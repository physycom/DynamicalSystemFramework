/// @file       /src/dsf/headers/RoadNetwork.hpp
/// @file       /src/dsf/headers/RoadNetwork.hpp
/// @brief      Defines the RoadNetwork class.
///
/// @details    This file contains the definition of the RoadNetwork class.
///             The RoadNetwork class represents a graph in the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include "AdjacencyMatrix.hpp"
#include "Network.hpp"
#include "RoadJunction.hpp"
#include "Intersection.hpp"
#include "TrafficLight.hpp"
#include "Roundabout.hpp"
#include "Station.hpp"
#include "Street.hpp"
#include "../utility/Typedef.hpp"
#include "../utility/TypeTraits/is_node.hpp"
#include "../utility/TypeTraits/is_street.hpp"

#include <algorithm>
#include <concepts>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <type_traits>
#include <utility>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include <format>

#include <spdlog/spdlog.h>

namespace dsf {
  /// @brief The RoadNetwork class represents a graph in the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  class RoadNetwork : public Network<RoadJunction, Street> {
  private:
    unsigned long long m_capacity;

    /// @brief If every node has coordinates, set the street angles
    /// @details The street angles are set using the node's coordinates.
    void m_setStreetAngles();

    void m_updateMaxAgentCapacity();

    void m_csvEdgesImporter(std::ifstream& file, const char separator = ';');
    void m_csvNodePropertiesImporter(std::ifstream& file, const char separator = ';');

    void m_jsonEdgesImporter(std::ifstream& file, const bool bCreateInverse = false);

  public:
    RoadNetwork();
    /// @brief Construct a new RoadNetwork object
    /// @param adj An adjacency matrix made by a SparseMatrix representing the graph's adjacency matrix
    RoadNetwork(AdjacencyMatrix const& adj);

    /// @brief Get the graph's number of coil streets
    /// @return The number of coil streets
    Size nCoils() const;

    /// @brief Get the graph's number of intersections
    /// @return The number of intersections
    Size nIntersections() const;
    /// @brief Get the graph's number of roundabouts
    /// @return The number of roundabouts
    Size nRoundabouts() const;
    /// @brief Get the graph's number of traffic lights
    /// @return The number of traffic lights
    Size nTrafficLights() const;

    /// @brief Adjust the nodes' transport capacity
    /// @details The nodes' capacity is adjusted using the graph's streets transport capacity, which may vary basing on the number of lanes. The node capacity will be set to the sum of the incoming streets' transport capacity.
    void adjustNodeCapacities();
    /// @brief Initialize the traffic lights with random parameters
    /// @param minGreenTime The minimum green time for the traffic lights cycles (default is 30)
    /// @details Traffic Lights with no parameters set are initialized with random parameters.
    /// Street priorities are assigned considering the number of lanes and the speed limit.
    /// Traffic Lights with an input degree lower than 3 are converted to standard intersections.
    void initTrafficLights(Delay const minGreenTime = 30);
    /// @brief Automatically re-maps street lanes basing on network's topology
    /// @details For example, if one street has the right turn forbidden, then the right lane becomes a straight one
    void autoMapStreetLanes();

    /// @brief Import the graph's adjacency matrix from a file.
    /// If the file is not of a supported format, it will read the file as a matrix with the first two elements being
    /// the number of rows and columns and the following elements being the matrix elements.
    /// @param fileName The name of the file to import the adjacency matrix from.
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @param defaultSpeed The default speed limit for the streets
    /// @throws std::invalid_argument if the file is not found or invalid
    /// The matrix format is deduced from the file extension. Currently only .dsm files are supported.
    void importMatrix(const std::string& fileName,
                      bool isAdj = true,
                      double defaultSpeed = 13.8888888889);
    /// @brief Import the graph's nodes from a file
    /// @param fileName The name of the file to import the nodes from.
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    /// @details The file format is deduced from the file extension. Currently only .dsm files are supported.
    ///           The first input number is the number of nodes, followed by the coordinates of each node.
    ///           In the i-th row of the file, the (i - 1)-th node's coordinates are expected.
    [[deprecated]] void importCoordinates(const std::string& fileName);

    /// @brief Import the graph's streets from a file
    /// @param fileName The name of the file to import the streets from.
    /// @details Supports csv, json and geojson file formats.
    /// The file format is deduced from the file extension.
    /// Supported fields:
    /// - id: The id of the street
    /// - source: The id of the source node
    /// - target: The id of the target node
    /// - length: The length of the street, in meters
    /// - nlanes: The number of lanes of the street
    /// - maxspeed: The street's speed limit, in km/h
    /// - name: The name of the street
    /// - geometry: The geometry of the street, as a LINESTRING
    ///
    ///   Next columns are optional (meaning that their absence will not -hopefully- cause any pain):
    ///
    /// - type: The type of the street (e.g. residential, primary, secondary, etc.)
    /// - forbiddenTurns: The forbidden turns of the street, encoding information about street into which the street cannot output agents. The format is a string "sourceId1-targetid1, sourceId2-targetid2,..."
    /// - coilcode: An integer code to identify the coil located on the street
    /// - customWeight: will be stored in the `weight` parameter of the Edge class. You can use it for the shortest path via dsf::weight_functions::customWeight.
    template <typename... TArgs>
    void importEdges(const std::string& fileName, TArgs&&... args);
    /// @brief Import the graph's nodes properties from a file
    /// @param fileName The name of the file to import the nodes properties from.
    /// @details Supports csv file format. Please specify the separator as second parameter.
    /// Supported fields:
    /// - id: The id of the node
    /// - type: The type of the node, e.g. roundabout, traffic_signals, etc.
    /// - geometry: The geometry of the node, as a POINT
    template <typename... TArgs>
    void importNodeProperties(const std::string& fileName, TArgs&&... args);
    /// @brief Import the graph's traffic lights from a file
    /// @param fileName The name of the file to import the traffic lights from.
    /// @details The file format is csv-like with the ';' separator. Supported columns (in order):
    /// - id: The id of the TrafficLight node
    /// - sourceId: The id of the source node of the incoming street
    /// - cycleTime: The traffic light's cycle time, in seconds
    /// - greenTime: The green time of the considered phase, in time-steps
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    /// @details The traffic lights are imported from the specified file. If the file does not provide
    ///           sufficient parameters, the behavior of the traffic light initialization is undefined.
    ///           Ensure the file contains valid and complete data for accurate traffic light configuration.
    ///           Street priorities may be assigned based on additional parameters such as the number of lanes
    ///           and the speed limit, if such data is available in the file.
    void importTrafficLights(const std::string& fileName);

    /// @brief Export the graph's adjacency matrix to a file
    /// @param path The path to the file to export the adjacency matrix to (default: ./matrix.dsm)
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    [[deprecated]] void exportMatrix(std::string path = "./matrix.dsm",
                                     bool isAdj = true);

    template <typename T1, typename... Tn>
      requires is_node_v<std::remove_reference_t<T1>> &&
               (is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(T1&& node, Tn&&... nodes);

    /// @brief Convert an existing node to a traffic light
    /// @param nodeId The id of the node to convert to a traffic light
    /// @param cycleTime The traffic light's cycle time
    /// @param counter The traffic light's counter initial value. Default is 0
    /// @return A reference to the traffic light
    /// @throws std::invalid_argument if the node does not exist
    TrafficLight& makeTrafficLight(Id const nodeId,
                                   Delay const cycleTime,
                                   Delay const counter = 0);
    /// @brief Convert an existing node into a roundabout
    /// @param nodeId The id of the node to convert to a roundabout
    /// @return A reference to the roundabout
    /// @throws std::invalid_argument if the node does not exist
    Roundabout& makeRoundabout(Id nodeId);

    void makeStochasticStreet(Id streetId, double const flowRate);
    /// @brief Convert an existing street into a spire street
    /// @param streetId The id of the street to convert to a spire street
    /// @throws std::invalid_argument if the street does not exist
    void makeSpireStreet(Id streetId);
    /// @brief Convert an existing node into a station
    /// @param nodeId The id of the node to convert to a station
    /// @param managementTime The station's management time
    /// @return A reference to the station
    /// @throws std::invalid_argument if the node does not exist
    Station& makeStation(Id nodeId, const unsigned int managementTime);

    /// @brief Add a street to the graph
    /// @param street A reference to the street to add
    void addStreet(Street&& street);

    template <typename T1>
      requires is_street_v<std::remove_reference_t<T1>>
    void addStreets(T1&& street);

    template <typename T1, typename... Tn>
      requires is_street_v<std::remove_reference_t<T1>> &&
               (is_street_v<std::remove_reference_t<Tn>> && ...)
    void addStreets(T1&& street, Tn&&... streets);

    /// @brief Get a street from the graph
    /// @param source The source node
    /// @param destination The destination node
    /// @return A std::unique_ptr to the street if it exists, nullptr otherwise
    const std::unique_ptr<Street>* street(Id source, Id destination) const;

    /// @brief Get the maximum agent capacity
    /// @return unsigned long long The maximum agent capacity of the graph
    inline auto capacity() const noexcept { return m_capacity; }

    /// @brief Perform a global Dijkstra search to a target node from all other nodes in the graph
    /// @tparam DynamicsFunc A callable type that takes a const reference to a Street and returns a double representing the edge weight
    /// @param targetId The id of the target node
    /// @param getEdgeWeight A callable that takes a const reference to a Street and returns a double representing the edge weight
    /// @param threshold A threshold value to consider alternative paths
    /// @return A map where each key is a node id and the value is a vector of next hop node ids toward the target
    /// @throws std::out_of_range if the target node does not exist
    /// @throws std::invalid_argument if the dynamics function is not callable with a const reference
    template <typename DynamicsFunc>
      requires(std::is_invocable_r_v<double, DynamicsFunc, std::unique_ptr<Street> const&>)
    std::unordered_map<Id, std::vector<Id>> allPathsTo(
        Id const targetId,
        DynamicsFunc getEdgeWeight,
        double const threshold = 1e-9) const;
  };

  template <typename... TArgs>
  void RoadNetwork::importEdges(const std::string& fileName, TArgs&&... args) {
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    auto const fileExt = fileName.substr(fileName.find_last_of('.') + 1);
    if (!fileExtMap.contains(fileExt)) {
      throw std::invalid_argument(
          std::format("File extension ({}) not supported", fileExt));
    }
    switch (fileExtMap.at(fileExt)) {
      case FileExt::CSV:
        spdlog::debug("Importing nodes from CSV file: {}", fileName);
        this->m_csvEdgesImporter(file, std::forward<TArgs>(args)...);
        break;
      case FileExt::GEOJSON:
      case FileExt::JSON:
        spdlog::debug("Importing nodes from JSON file: {}", fileName);
        this->m_jsonEdgesImporter(file, std::forward<TArgs>(args)...);
        break;
      default:
        throw std::invalid_argument(
            std::format("File extension ({}) not supported", fileExt));
    }

    spdlog::debug("Successfully imported {} edges", this->nEdges());
  }
  template <typename... TArgs>
  void RoadNetwork::importNodeProperties(const std::string& fileName, TArgs&&... args) {
    if (this->nNodes() == 0) {
      throw std::runtime_error(
          "Cannot import node properties when there are no nodes in the network. Please "
          "import edges or construct network first.");
    }
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    auto const fileExt = fileName.substr(fileName.find_last_of('.') + 1);
    if (!fileExtMap.contains(fileExt)) {
      throw std::invalid_argument(
          std::format("File extension ({}) not supported", fileExt));
    }
    switch (fileExtMap.at(fileExt)) {
      case FileExt::CSV:
        spdlog::debug("Importing node properties from CSV file: {}", fileName);
        this->m_csvNodePropertiesImporter(file, std::forward<TArgs>(args)...);
        break;
      case FileExt::JSON:
      case FileExt::GEOJSON:
        throw std::invalid_argument(
            "Importing node properties from JSON or GEOJSON files is not supported.");
      default:
        throw std::invalid_argument(
            std::format("File extension ({}) not supported", fileExt));
    }
    spdlog::debug("Successfully imported node properties for {} nodes", nNodes());
  }

  template <typename T1, typename... Tn>
    requires is_node_v<std::remove_reference_t<T1>> &&
             (is_node_v<std::remove_reference_t<Tn>> && ...)
  void RoadNetwork::addNodes(T1&& node, Tn&&... nodes) {
    addNode(std::forward<T1>(node));
    addNodes(std::forward<Tn>(nodes)...);
  }

  template <typename T1>
    requires is_street_v<std::remove_reference_t<T1>>
  void RoadNetwork::addStreets(T1&& street) {
    addStreet(std::move(street));
  }

  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> &&
             (is_street_v<std::remove_reference_t<Tn>> && ...)
  void RoadNetwork::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::move(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename DynamicsFunc>
    requires(std::is_invocable_r_v<double, DynamicsFunc, std::unique_ptr<Street> const&>)
  std::unordered_map<Id, std::vector<Id>> RoadNetwork::allPathsTo(
      Id const sourceId, DynamicsFunc f, double const threshold) const {
    // Check if source node exists
    auto const& nodes = this->nodes();
    if (!nodes.contains(sourceId)) {
      throw std::out_of_range(
          std::format("Source node with id {} does not exist.", sourceId));
    }

    // Distance from each node to the source (going backward)
    std::unordered_map<Id, double> distToSource;
    distToSource.reserve(nNodes());
    // Next hop from each node toward the source
    std::unordered_map<Id, std::vector<Id>> nextHopsToSource;

    // Priority queue: pair<distance, nodeId> (min-heap)
    std::priority_queue<std::pair<double, Id>,
                        std::vector<std::pair<double, Id>>,
                        std::greater<>>
        pq;

    // Initialize all nodes with infinite distance
    std::for_each(nodes.cbegin(), nodes.cend(), [&](auto const& pair) {
      distToSource[pair.first] = std::numeric_limits<double>::infinity();
      nextHopsToSource[pair.first] = std::vector<Id>();
    });

    // Source has distance 0 to itself
    distToSource[sourceId] = 0.0;
    pq.push({0.0, sourceId});

    while (!pq.empty()) {
      auto [currentDist, currentNode] = pq.top();
      pq.pop();

      // Skip if we've already found a better path to this node
      if (currentDist > distToSource[currentNode]) {
        continue;
      }

      // Explore all incoming edges (nodes that can reach currentNode)
      auto const& inEdges = node(currentNode)->ingoingEdges();
      for (auto const& inEdgeId : inEdges) {
        Id neighborId = edge(inEdgeId)->source();

        // Calculate the weight of the edge from neighbor to currentNode using the dynamics function
        double edgeWeight = f(this->edge(inEdgeId));
        double newDistToSource = distToSource[currentNode] + edgeWeight;

        // If we found a shorter path from neighborId to source
        if (newDistToSource < distToSource[neighborId]) {
          distToSource[neighborId] = newDistToSource;
          nextHopsToSource[neighborId].clear();
          nextHopsToSource[neighborId].push_back(currentNode);
          pq.push({newDistToSource, neighborId});
        }
        // If we found an equally good path, add it as alternative
        else if (newDistToSource < (1. + threshold) * distToSource[neighborId]) {
          spdlog::debug(
              "Found alternative path to node {} with distance {:.6f} (existing: {:.6f}) "
              "for threshold {:.6f}",
              neighborId,
              newDistToSource,
              distToSource[neighborId],
              threshold);
          // Check if currentNode is not already in the nextHops
          auto& hops = nextHopsToSource[neighborId];
          if (std::find(hops.begin(), hops.end(), currentNode) == hops.end()) {
            hops.push_back(currentNode);
          }
        }
      }
    }

    // Build result: only include reachable nodes (excluding source)
    std::unordered_map<Id, std::vector<Id>> result;
    for (auto const& [nodeId, hops] : nextHopsToSource) {
      if (nodeId != sourceId &&
          distToSource[nodeId] != std::numeric_limits<double>::infinity() &&
          !hops.empty()) {
        result[nodeId] = hops;
      }
    }

    return result;
  }
};  // namespace dsf
