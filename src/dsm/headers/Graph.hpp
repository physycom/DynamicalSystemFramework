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

#include "AdjacencyMatrix.hpp"
#include "DijkstraWeights.hpp"
#include "Node.hpp"
#include "Intersection.hpp"
#include "TrafficLight.hpp"
#include "Roundabout.hpp"
#include "Station.hpp"
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
    std::unordered_map<std::string, Id> m_nodeMapping;
    AdjacencyMatrix m_adjacencyMatrix;
    std::vector<Id> m_inputNodes;
    std::vector<Id> m_outputNodes;
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
    Graph(AdjacencyMatrix const& adj);
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
      m_adjacencyMatrix = other.m_adjacencyMatrix;
      m_inputNodes = other.m_inputNodes;
      m_outputNodes = other.m_outputNodes;
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
      m_adjacencyMatrix = other.m_adjacencyMatrix;
      m_inputNodes = other.m_inputNodes;
      m_outputNodes = other.m_outputNodes;

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
    /// @param path The path to the file to export the adjacency matrix to (default: ./matrix.dsm)
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    void exportMatrix(std::string path = "./matrix.dsm", bool isAdj = true);
    /// @brief Export the nodes' coordinates to a file
    /// @param path The path to the file to export the nodes' coordinates to (default: ./nodes.dsm)
    void exportCoordinates(std::string const& path = "./coordinates.csv");

    /// @brief Add a node to the graph
    /// @param node A std::unique_ptr to the node to add
    void addNode(std::unique_ptr<Node> node);
    /// @brief Add a node of type node_t to the graph
    /// @param id The node's id
    /// @param args The node's arguments to forward to the constructor
    /// @return A reference to the added node
    template <typename node_t, typename... TArgs>
      requires(std::is_base_of_v<Node, node_t>,
               std::constructible_from<node_t, Id, TArgs...>)
    node_t& addNode(Id id, TArgs&&... args);

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

    StochasticStreet& makeStochasticStreet(Id streetId, double const flowRate);
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

    /// @brief Add an edge of type edge_t to the graph
    /// @param edge A std::unique_ptr to the edge to add
    /// @param args The edge's arguments to forward to the constructor
    /// @return A reference to the added edge
    template <typename edge_t, typename... TArgs>
      requires(std::is_base_of_v<Street, edge_t>,
               std::constructible_from<edge_t, Id, TArgs...>)
    edge_t& addEdge(Id id, TArgs&&... args);
    /// @brief Add a street to the graph
    /// @param street A std::unique_ptr to the street to add
    void addStreet(std::unique_ptr<Street> street);
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
    /// @return A const reference to the graph's adjacency matrix
    AdjacencyMatrix const& adjMatrix() const { return m_adjacencyMatrix; }
    /// @brief Get the graph's number of nodes
    /// @return size_t The number of nodes in the graph
    size_t nNodes() const { return m_nodes.size(); }
    /// @brief Get the graph's node map
    /// @return A std::unordered_map containing the graph's nodes
    const std::unordered_map<Id, std::unique_ptr<Node>>& nodeSet() const {
      return m_nodes;
    }
    /// @brief Get the graph's node map
    /// @return A std::unordered_map containing the graph's nodes
    std::unordered_map<Id, std::unique_ptr<Node>>& nodeSet() { return m_nodes; }
    /// @brief Get a node from the graph
    /// @param id The node's id
    const std::unique_ptr<Node>& node(Id id) const { return m_nodes.at(id); }
    /// @brief Get the Graph's number of streets
    /// @return size_t The number of streets in the graph
    size_t nEdges() const { return m_streets.size(); }
    /// @brief Get the graph's street map
    /// @return A std::unordered_map containing the graph's streets
    const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet() const {
      return m_streets;
    }
    /// @brief Get the graph's street map
    /// @return A std::unordered_map containing the graph's streets
    std::unordered_map<Id, std::unique_ptr<Street>>& streetSet() { return m_streets; }
    /// @brief Get a street from the graph
    /// @param id The street's id
    const std::unique_ptr<Street>& street(Id id) const { return m_streets.at(id); }
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

    std::vector<Id> const& inputNodes() const { return m_inputNodes; }
    std::vector<Id> const& outputNodes() const { return m_outputNodes; }

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node
    /// @param destination The destination node
    /// @return A DijkstraResult object containing the path and the distance
    template <typename Func = std::function<double(const Graph*, Id, Id)>>
      requires(std::is_same_v<std::invoke_result_t<Func, const Graph*, Id, Id>, double>)
    std::optional<DijkstraResult> shortestPath(const Node& source,
                                               const Node& destination,
                                               Func f = streetLength) const;

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node id
    /// @param destination The destination node id
    /// @return A DijkstraResult object containing the path and the distance
    template <typename Func = std::function<double(const Graph*, Id, Id)>>
      requires(std::is_same_v<std::invoke_result_t<Func, const Graph*, Id, Id>, double>)
    std::optional<DijkstraResult> shortestPath(Id source,
                                               Id destination,
                                               Func f = streetLength) const;
  };

  template <typename node_t, typename... TArgs>
    requires(std::is_base_of_v<Node, node_t>,
             std::constructible_from<node_t, Id, TArgs...>)
  node_t& Graph::addNode(Id id, TArgs&&... args) {
    addNode(std::make_unique<node_t>(id, std::forward<TArgs>(args)...));
    return dynamic_cast<node_t&>(*m_nodes.at(id));
  }
  template <typename T1, typename... Tn>
    requires is_node_v<std::remove_reference_t<T1>> &&
             (is_node_v<std::remove_reference_t<Tn>> && ...)
  void Graph::addNodes(T1&& node, Tn&&... nodes) {
    addNode(std::forward<T1>(node));
    addNodes(std::forward<Tn>(nodes)...);
  }

  template <typename edge_t, typename... TArgs>
    requires(std::is_base_of_v<Street, edge_t>,
             std::constructible_from<edge_t, Id, TArgs...>)
  edge_t& Graph::addEdge(Id id, TArgs&&... args) {
    addStreet(std::make_unique<edge_t>(id, std::forward<TArgs>(args)...));
    return dynamic_cast<edge_t&>(*m_streets.at(id));
  }

  template <typename T1>
    requires is_street_v<std::remove_reference_t<T1>>
  void Graph::addStreets(T1&& street) {
    if (m_streets.contains(street.id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} already exists.", street.id())));
    }
    // emplace nodes
    auto const srcId{street.source()};
    auto const dstId{street.target()};
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

  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> &&
             (is_street_v<std::remove_reference_t<Tn>> && ...)
  void Graph::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::forward<T1>(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename Func>
    requires(std::is_same_v<std::invoke_result_t<Func, const Graph*, Id, Id>, double>)
  std::optional<DijkstraResult> Graph::shortestPath(const Node& source,
                                                    const Node& destination,
                                                    Func f) const {
    return this->shortestPath(source.id(), destination.id());
  }

  template <typename Func>
    requires(std::is_same_v<std::invoke_result_t<Func, const Graph*, Id, Id>, double>)
  std::optional<DijkstraResult> Graph::shortestPath(Id source,
                                                    Id destination,
                                                    Func getStreetWeight) const {
    const Id sourceId{source};

    std::unordered_set<Id> unvisitedNodes;
    bool source_found{false};
    bool dest_found{false};
    std::for_each(m_nodes.begin(),
                  m_nodes.end(),
                  [&unvisitedNodes, &source_found, &dest_found, source, destination](
                      const auto& node) -> void {
                    if (!source_found && node.first == source) {
                      source_found = true;
                    }
                    if (!dest_found && node.first == destination) {
                      dest_found = true;
                    }
                    unvisitedNodes.emplace(node.first);
                  });
    if (!source_found || !dest_found) {
      return std::nullopt;
    }

    const size_t n_nodes{m_nodes.size()};
    auto adj{m_adjacencyMatrix};

    std::unordered_set<Id> visitedNodes;
    std::vector<std::pair<Id, double>> dist(n_nodes);
    std::for_each(dist.begin(), dist.end(), [count = 0](auto& element) mutable -> void {
      element.first = count;
      element.second = std::numeric_limits<double>::max();
      ++count;
    });
    dist[source] = std::make_pair(source, 0.);

    std::vector<std::pair<Id, double>> prev(n_nodes);
    std::for_each(prev.begin(), prev.end(), [](auto& pair) -> void {
      pair.first = std::numeric_limits<Id>::max();
      pair.second = std::numeric_limits<double>::max();
    });
    prev[source].second = 0.;

    while (unvisitedNodes.size() != 0) {
      source = *std::min_element(unvisitedNodes.begin(),
                                 unvisitedNodes.end(),
                                 [&dist](const auto& a, const auto& b) -> bool {
                                   return dist[a].second < dist[b].second;
                                 });

      unvisitedNodes.erase(source);
      visitedNodes.emplace(source);
      auto const& neighbors{adj.getRow(source)};
      for (auto const& neighbour : neighbors) {
        // if the node has already been visited, skip it
        if (visitedNodes.find(neighbour) != visitedNodes.end()) {
          continue;
        }
        double streetWeight = getStreetWeight(this, source, neighbour);
        // if current path is shorter than the previous one, update the distance
        if (streetWeight + dist[source].second < dist[neighbour].second) {
          dist[neighbour].second = streetWeight + dist[source].second;
          prev[neighbour] = std::make_pair(source, dist[neighbour].second);
        }
      }

      adj.clearCol(source);
    }

    std::vector<Id> path{destination};
    Id previous{destination};
    while (true) {
      previous = prev[previous].first;
      if (previous == std::numeric_limits<Id>::max()) {
        return std::nullopt;
      }
      path.push_back(previous);
      if (previous == sourceId) {
        break;
      }
    }

    std::reverse(path.begin(), path.end());
    return DijkstraResult(path, prev[destination].second);
  }
};  // namespace dsm
