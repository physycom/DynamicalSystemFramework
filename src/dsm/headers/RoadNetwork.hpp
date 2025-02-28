/// @file       /src/dsm/headers/RoadNetwork.hpp
/// @file       /src/dsm/headers/RoadNetwork.hpp
/// @brief      Defines the RoadNetwork class.
///
/// @details    This file contains the definition of the RoadNetwork class.
///             The RoadNetwork class represents a graph in the network. It is templated by the type
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
#include "Network.hpp"
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

  /// @brief The RoadNetwork class represents a graph in the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  class RoadNetwork : public Network<Node, Street> {
  private:
    std::unordered_map<std::string, Id> m_nodeMapping;
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

    void m_addMissingNodes(Id const nodeId) final;

  public:
    RoadNetwork();
    /// @brief Construct a new RoadNetwork object
    /// @param adj An adjacency matrix made by a SparseMatrix representing the graph's adjacency matrix
    RoadNetwork(AdjacencyMatrix const& adj);
    /// @brief Construct a new RoadNetwork object
    /// @param streetSet A map of streets representing the graph's streets
    RoadNetwork(const std::unordered_map<Id, std::unique_ptr<Street>>& streetSet);

    RoadNetwork(const RoadNetwork& other) : Network{AdjacencyMatrix()} {
      std::for_each(other.m_nodes.begin(), other.m_nodes.end(), [this](const auto& pair) {
        this->m_nodes.emplace(pair.first, pair.second.get());
      });
      std::for_each(other.m_edges.begin(), other.m_edges.end(), [this](const auto& pair) {
        this->m_edges.emplace(pair.first, pair.second.get());
      });
      m_nodeMapping = other.m_nodeMapping;
      m_adjacencyMatrix = other.m_adjacencyMatrix;
      m_inputNodes = other.m_inputNodes;
      m_outputNodes = other.m_outputNodes;
    }

    RoadNetwork& operator=(const RoadNetwork& other) {
      std::for_each(other.m_nodes.begin(), other.m_nodes.end(), [this](const auto& pair) {
        this->m_nodes.insert_or_assign(pair.first,
                                       std::unique_ptr<Node>(pair.second.get()));
      });
      std::for_each(other.m_edges.begin(), other.m_edges.end(), [this](const auto& pair) {
        this->m_edges.insert_or_assign(pair.first,
                                       std::make_unique<Street>(*pair.second));
      });
      m_nodeMapping = other.m_nodeMapping;
      m_adjacencyMatrix = other.m_adjacencyMatrix;
      m_inputNodes = other.m_inputNodes;
      m_outputNodes = other.m_outputNodes;

      return *this;
    }

    RoadNetwork(RoadNetwork&&) = default;
    RoadNetwork& operator=(RoadNetwork&&) = default;

    /// @brief Build the graph's adjacency matrix and computes max capacity
    /// @details The adjacency matrix is built using the graph's streets and nodes. N.B.: The street ids
    /// are reassigned using the max node id, i.e. newStreetId = srcId * n + dstId, where n is the max node id.
    /// Moreover, street angles and geometries are set using the nodes' coordinates.
    void buildAdj();
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

    /// @brief Export the graph's nodes to a csv-like file separated with ';'
    /// @param path The path to the file to export the nodes to
    /// @details The file format is csv-like, with the first line being the column names: id;lon;lat
    void exportNodes(const std::string& fileName);
    /// @brief Export the graph's edges to a csv-like file separated with ';'
    /// @param path The path to the file to export the edges to
    /// @details The file format is csv-like, with the first line being the column names: id;source_id;target_id;name;geometry
    void exportEdges(const std::string& fileName);
    /// @brief Export the graph's adjacency matrix to a file
    /// @param path The path to the file to export the adjacency matrix to (default: ./matrix.dsm)
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    void exportMatrix(std::string path = "./matrix.dsm", bool isAdj = true);

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

    /// @brief Get the input nodes of the graph
    /// @return std::vector<Id> const& The input nodes of the graph
    std::vector<Id> const& inputNodes() const { return m_inputNodes; }
    /// @brief Get the input nodes of the graph
    /// @return std::vector<Id>& The input nodes of the graph
    std::vector<Id>& inputNodes() { return m_inputNodes; }
    /// @brief Get the output nodes of the graph
    /// @return std::vector<Id> const& The output nodes of the graph
    std::vector<Id> const& outputNodes() const { return m_outputNodes; }
    /// @brief Get the output nodes of the graph
    /// @return std::vector<Id>& The output nodes of the graph
    std::vector<Id>& outputNodes() { return m_outputNodes; }

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node
    /// @param destination The destination node
    /// @return A DijkstraResult object containing the path and the distance
    template <typename Func = std::function<double(const RoadNetwork*, Id, Id)>>
      requires(
          std::is_same_v<std::invoke_result_t<Func, const RoadNetwork*, Id, Id>, double>)
    std::optional<DijkstraResult> shortestPath(
        const Node& source,
        const Node& destination,
        Func f = weight_functions::streetLength) const;

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source The source node id
    /// @param destination The destination node id
    /// @return A DijkstraResult object containing the path and the distance
    template <typename Func = std::function<double(const RoadNetwork*, Id, Id)>>
      requires(
          std::is_same_v<std::invoke_result_t<Func, const RoadNetwork*, Id, Id>, double>)
    std::optional<DijkstraResult> shortestPath(
        Id source, Id destination, Func f = weight_functions::streetLength) const;
  };

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
    if (m_edges.contains(street.id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Street with id {} already exists.", street.id())));
    }
    addEdge<Street>(street.id(), street);
  }

  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> &&
             (is_street_v<std::remove_reference_t<Tn>> && ...)
  void RoadNetwork::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::forward<T1>(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename Func>
    requires(
        std::is_same_v<std::invoke_result_t<Func, const RoadNetwork*, Id, Id>, double>)
  std::optional<DijkstraResult> RoadNetwork::shortestPath(const Node& source,
                                                          const Node& destination,
                                                          Func f) const {
    return this->shortestPath(source.id(), destination.id());
  }

  template <typename Func>
    requires(
        std::is_same_v<std::invoke_result_t<Func, const RoadNetwork*, Id, Id>, double>)
  std::optional<DijkstraResult> RoadNetwork::shortestPath(Id source,
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
