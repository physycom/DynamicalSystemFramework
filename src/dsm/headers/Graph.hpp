/// @file       /src/dsm/headers/Graph.hpp
/// @file       /src/dsm/headers/Graph.hpp
/// @brief      Defines the Graph class.
///
/// @details    This file contains the definition of the Graph class.
///             The Graph class represents a graph in the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

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

#include "Node.hpp"
#include "SparseMatrix.hpp"
#include "Street.hpp"
#include "../utility/DijkstraResult.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"
#include "../utility/TypeTraits/is_node.hpp"
#include "../utility/TypeTraits/is_street.hpp"

namespace dsm {

  /// @brief The Graph class represents a graph in the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  class Graph {
  private:
    std::unordered_map<Id, std::unique_ptr<Node>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<Street>> m_streets;
    std::unordered_map<Id, Id> m_nodeMapping;
    SparseMatrix<bool> m_adjacency;
    unsigned long long m_maxAgentCapacity;

    /// @brief Reassign the street ids using the max node id
    /// @details The street ids are reassigned using the max node id, i.e.
    /// newStreetId = srcId * n + dstId, where n is the max node id.
    void m_reassignIds();
    /// @brief If every node has coordinates, set the street angles
    /// @details The street angles are set using the node's coordinates.
    void m_setStreetAngles();

  public:
    Graph();
    /// @brief Construct a new Graph object
    /// @param adj An adjacency matrix made by a SparseMatrix representing the graph's adjacency matrix
    Graph(const SparseMatrix<bool>& adj);
    /// @brief Construct a new Graph object
    /// @param streetSet A map of streets representing the graph's streets
    Graph(const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet);

    Graph(const Graph& other) {
      std::for_each(other.m_nodes.begin(), other.m_nodes.end(), [this](const auto& pair) {
        this->m_nodes.emplace(pair.first, pair.second.get());
      });
      std::for_each(
          other.m_streets.begin(), other.m_streets.end(), [this](const auto& pair) {
            this->m_streets.emplace(pair.first, pair.second.get());
          });
      m_nodeMapping = other.m_nodeMapping;
      m_adjacency = other.m_adjacency;
    }

    Graph& operator=(const Graph& other) {
      std::for_each(other.m_nodes.begin(), other.m_nodes.end(), [this](const auto& pair) {
        this->m_nodes.insert_or_assign(pair.first,
                                       std::unique_ptr<Node>(pair.second.get()));
      });
      std::for_each(
          other.m_streets.begin(), other.m_streets.end(), [this](const auto& pair) {
            this->m_streets.insert_or_assign(pair.first,
                                             std::make_unique<Street>(*pair.second));
          });
      m_nodeMapping = other.m_nodeMapping;
      m_adjacency = other.m_adjacency;

      return *this;
    }

    Graph(Graph&&) = default;
    Graph& operator=(Graph&&) = default;

    /// @brief Build the graph's adjacency matrix and computes max capacity
    /// @details The adjacency matrix is built using the graph's streets and nodes. N.B.: The street ids
    /// are reassigned using the max node id, i.e. newStreetId = srcId * n + dstId, where n is the max node id.
    void buildAdj();
    /// @brief Build the graph's street angles using the node's coordinates
    void buildStreetAngles();
    /// @brief Adjust the nodes' transport capacity
    /// @details The nodes' capacity is adjusted using the graph's streets transport capacity, which may vary basing on the number of lanes. The node capacity will be set to the sum of the incoming streets' transport capacity.
    void adjustNodeCapacities();
    /// @brief Normalize the streets' capacities
    /// @param meanVehicleLength The mean vehicle length
    /// @details The streets' capacities are normalized using the mean vehicle length following the formula:
    /// \f$ \text{capacity} = \frac{\text{length} * \text{nLanes}}{\text{meanVehicleLength}} \f$
    void normalizeStreetCapacities(double meanVehicleLength = 5.);

    /// @brief Import the graph's adjacency matrix from a file.
    /// If the file is not of a supported format, it will read the file as a matrix with the first two elements being
    /// the number of rows and columns and the following elements being the matrix elements.
    /// @param fileName The name of the file to import the adjacency matrix from.
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    /// The matrix format is deduced from the file extension. Currently only .dsm files are supported.
    void importMatrix(const std::string& fileName, bool isAdj = true);
    /// @brief Import the graph's nodes from a file
    /// @param fileName The name of the file to import the nodes from.
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    /// @details The file format is deduced from the file extension. Currently only .dsm files are supported.
    ///           The first input number is the number of nodes, followed by the coordinates of each node.
    ///           In the i-th row of the file, the (i - 1)-th node's coordinates are expected.
    void importCoordinates(const std::string& fileName);
    /// @brief Import the graph's nodes from a file
    /// @param fileName The name of the file to import the nodes from.
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    void importOSMNodes(const std::string& fileName);
    /// @brief Import the graph's streets from a file
    /// @param fileName The name of the file to import the streets from.
    /// @throws std::invalid_argument if the file is not found, invalid or the format is not supported
    void importOSMEdges(const std::string& fileName);

    /// @brief Export the graph's adjacency matrix to a file
    /// @param path The path to the file to export the adjacency matrix to.
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    void exportMatrix(std::string path = "./matrix.dsm", bool isAdj = true);

    /// @brief Add a node to the graph
    /// @param node A std::unique_ptr to the node to add
    void addNode(std::unique_ptr<Node> node);
    /// @brief Add a node to the graph
    /// @param node A reference to the node to add
    void addNode(const Intersection& node);

    template <typename... Tn>
      requires(is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(Tn&&... nodes);

    template <typename T1, typename... Tn>
      requires is_node_v<std::remove_reference_t<T1>> &&
               (is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(T1&& node, Tn&&... nodes);

    /// @brief Convert an existing node to a traffic light
    /// @tparam Delay The type of the traffic light's delay
    /// @param nodeId The id of the node to convert to a traffic light
    /// @throws std::invalid_argument if the node does not exist
    template <typename Delay>
      requires(std::unsigned_integral<Delay>)
    void makeTrafficLight(Id nodeId);
    /// @brief Convert an existing node into a roundabout
    /// @param nodeId The id of the node to convert to a roundabout
    /// @throws std::invalid_argument if the node does not exist
    void makeRoundabout(Id nodeId);
    /// @brief Convert an existing street into a spire street
    /// @param streetId The id of the street to convert to a spire street
    /// @throws std::invalid_argument if the street does not exist
    void makeSpireStreet(Id streetId);

    /// @brief Add a street to the graph
    /// @param street A std::shared_ptr to the street to add
    void addStreet(std::shared_ptr<Street> street);
    /// @brief Add a street to the graph
    /// @param street A reference to the street to add
    void addStreet(const Street& street);

    template <typename T1>
      requires is_street_v<std::remove_reference_t<T1>>
    void addStreets(T1&& street);

    template <typename T1, typename... Tn>
      requires is_street_v<std::remove_reference_t<T1>> &&
               (is_street_v<std::remove_reference_t<Tn>> && ...)
    void addStreets(T1&& street, Tn&&... streets);

    /// @brief Get the graph's adjacency matrix
    /// @return A std::shared_ptr to the graph's adjacency matrix
    const SparseMatrix<bool>& adjMatrix() const { return m_adjacency; }
    /// @brief Get the graph's node map
    /// @return A std::unordered_map containing the graph's nodes
    const std::unordered_map<Id, std::unique_ptr<Node>>& nodeSet() const {
      return m_nodes;
    }
    /// @brief Get the graph's node map
    /// @return A std::unordered_map containing the graph's nodes
    std::unordered_map<Id, std::unique_ptr<Node>>& nodeSet() { return m_nodes; }
    /// @brief Get the graph's street map
    /// @return A std::unordered_map containing the graph's streets
    const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet() const {
      return m_streets;
    }
    /// @brief Get the graph's street map
    /// @return A std::unordered_map containing the graph's streets
    std::unordered_map<Id, std::unique_ptr<Street>>& streetSet() { return m_streets; }
    /// @brief Get a street from the graph
    /// @param source The source node
    /// @param destination The destination node
    /// @return A std::unique_ptr to the street if it exists, nullptr otherwise
    const std::unique_ptr<Street>* street(Id source, Id destination) const;
    /// @brief Get the opposite street of a street in the graph
    /// @param streetId The id of the street
    /// @throws std::invalid_argument if the street does not exist
    /// @return A std::unique_ptr to the street if it exists, nullptr otherwise
    const std::unique_ptr<Street>* oppositeStreet(Id streetId) const;

    /// @brief Get the maximum agent capacity
    /// @return unsigned long long The maximum agent capacity of the graph
    unsigned long long maxCapacity() const { return m_maxAgentCapacity; }

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node
    /// @param destination The destination node
    /// @return A DijkstraResult object containing the path and the distance
    std::optional<DijkstraResult> shortestPath(const Intersection& source,
                                               const Intersection& destination) const;
    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node id
    /// @param destination The destination node id
    /// @return A DijkstraResult object containing the path and the distance
    std::optional<DijkstraResult> shortestPath(Id source, Id destination) const;
  };

  template <typename... Tn>
    requires(is_node_v<std::remove_reference_t<Tn>> && ...)
  void Graph::addNodes(Tn&&... nodes) {}

  template <typename T1, typename... Tn>
    requires is_node_v<std::remove_reference_t<T1>> &&
             (is_node_v<std::remove_reference_t<Tn>> && ...)
  void Graph::addNodes(T1&& node, Tn&&... nodes) {
    addNode(std::forward<T1>(node));
    addNodes(std::forward<Tn>(nodes)...);
  }

  template <typename T1>
    requires is_street_v<std::remove_reference_t<T1>>
  void Graph::addStreets(T1&& street) {
    if (m_streets.contains(street.id())) {
      throw std::invalid_argument(
          buildLog(std::format("Street with id {} already exists.", street.id())));
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

  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> &&
             (is_street_v<std::remove_reference_t<Tn>> && ...)
  void Graph::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::forward<T1>(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void Graph::makeTrafficLight(Id nodeId) {
    if (!m_nodes.contains(nodeId)) {
      throw std::invalid_argument(buildLog("Node does not exist."));
    }
    auto& pNode = m_nodes[nodeId];
    pNode = std::make_unique<TrafficLight<Delay>>(*pNode);
  }

};  // namespace dsm
