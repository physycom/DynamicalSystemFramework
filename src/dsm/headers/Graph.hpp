/// @file       src/Graph.hpp
/// @brief      Defines the Graph class.
///
/// @details    This file contains the definition of the Graph class.
///             The Graph class represents a graph in the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#ifndef Graph_hpp
#define Graph_hpp

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

#include "Node.hpp"
#include "SparseMatrix.hpp"
#include "Street.hpp"
#include "../utility/DijkstraResult.hpp"
#include "../utility/TypeTraits/is_node.hpp"
#include "../utility/TypeTraits/is_street.hpp"

namespace dsm {

  // Alias for shared pointers
  template <typename T>
  using shared = std::shared_ptr<T>;
  using std::make_shared;

  /// @brief The Graph class represents a graph in the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  class Graph {
  private:
    std::unordered_map<Id, shared<Node<Id, Size>>> m_nodes;
    std::unordered_map<Id, shared<Street<Id, Size>>> m_streets;
    shared<SparseMatrix<Id, bool>> m_adjacency;

  public:
    Graph();
    /// @brief Construct a new Graph object
    /// @param adj, An adjacency matrix made by a SparseMatrix representing the graph's adjacency matrix
    Graph(const SparseMatrix<Id, bool>& adj);
    /// @brief Construct a new Graph object
    /// @param streetSet, A map of streets representing the graph's streets
    Graph(const std::unordered_map<Id, shared<Street<Id, Size>>>& streetSet);

    /// @brief Build the graph's adjacency matrix
    void buildAdj();

    /// @brief Import the graph's adjacency matrix from a file.
    /// If the file is not of a supported format, it will read the file as a matrix with the first two elements being
    /// the number of rows and columns and the following elements being the matrix elements.
    /// @param fileName, The name of the file to import the adjacency matrix from.
    /// @param isAdj A boolean value indicating if the file contains the adjacency matrix or the distance matrix.
    /// @throws std::invalid_argument if the file is not found or invalid
    /// The matrix format is deduced from the file extension. Currently only .dsm files are supported.
    void importMatrix(const std::string& fileName, bool isAdj = true);

    /// @brief Add a node to the graph
    /// @param node, A std::shared_ptr to the node to add
    void addNode(shared<Node<Id, Size>> node);
    /// @brief Add a node to the graph
    /// @param node, A reference to the node to add
    void addNode(const Node<Id, Size>& node);

    template <typename... Tn>
      requires(is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(Tn&&... nodes);

    template <typename T1, typename... Tn>
      requires is_node_v<std::remove_reference_t<T1>> && (is_node_v<std::remove_reference_t<Tn>> && ...)
    void addNodes(T1&& node, Tn&&... nodes);

    /// @brief Add a street to the graph
    /// @param street, A std::shared_ptr to the street to add
    void addStreet(shared<Street<Id, Size>> street);
    /// @brief Add a street to the graph
    /// @param street, A reference to the street to add
    void addStreet(const Street<Id, Size>& street);

    template <typename T1>
      requires is_street_v<std::remove_reference_t<T1>>
    void addStreets(T1&& street);

    template <typename T1, typename... Tn>
      requires is_street_v<std::remove_reference_t<T1>> && (is_street_v<std::remove_reference_t<Tn>> && ...)
    void addStreets(T1&& street, Tn&&... streets);

    /// @brief Get the graph's adjacency matrix
    /// @return A std::shared_ptr to the graph's adjacency matrix
    shared<SparseMatrix<Id, bool>> adjMatrix() const;
    /// @brief Get the graph's node map
    /// @return A std::unordered_map containing the graph's nodes
    std::unordered_map<Id, shared<Node<Id, Size>>> nodeSet() const;
    /// @brief Get the graph's street map
    /// @return A std::unordered_map containing the graph's streets
    std::unordered_map<Id, shared<Street<Id, Size>>> streetSet() const;

    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source, The source node
    /// @param destination, The destination node
    /// @return A DijkstraResult object containing the path and the distance
    std::optional<DijkstraResult<Id>> shortestPath(const Node<Id, Size>& source,
                                                   const Node<Id, Size>& destination) const;
    /// @brief Get the shortest path between two nodes using dijkstra algorithm
    /// @param source, The source node id
    /// @param destination, The destination node id
    /// @return A DijkstraResult object containing the path and the distance
    std::optional<DijkstraResult<Id>> shortestPath(Id source, Id destination) const;
  };

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  Graph<Id, Size>::Graph() : m_adjacency{make_shared<SparseMatrix<Id, bool>>()} {}

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  Graph<Id, Size>::Graph(const SparseMatrix<Id, bool>& adj)
      : m_adjacency{make_shared<SparseMatrix<Id, bool>>(adj)} {
    std::ranges::for_each(std::views::iota(0, (int)adj.getColDim()), [this](auto i) -> void {
      m_nodes.insert(std::make_pair(i, make_shared<Node<Id, Size>>(i)));
    });

    std::ranges::for_each(std::views::iota(0, (int)adj.size()), [this, adj](auto i) -> void {
      this->m_streets.insert(std::make_pair(
          i, make_shared<Street<Id, Size>>(i, std::make_pair(i / adj.getColDim(), i % adj.getColDim()))));
    });
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  Graph<Id, Size>::Graph(const std::unordered_map<Id, shared<Street<Id, Size>>>& streetSet)
      : m_adjacency{make_shared<SparseMatrix<Id, bool>>()} {
    for (const auto& street : streetSet) {
      m_streets.insert(std::make_pair(street->id(), street));

      Id node1 = street->nodePair().first;
      Id node2 = street->nodePair().second;
      m_nodes.insert(node1, make_shared<Node<Id, Size>>(node1));
      m_nodes.insert(node2, make_shared<Node<Id, Size>>(node2));
    }

    buildAdj();
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::buildAdj() {
    // find max values in streets node pairs
    const size_t maxNode{m_nodes.size()};
    m_adjacency->reshape(maxNode, maxNode);
    for (const auto& street : m_streets) {
      m_adjacency->insert(street.second->nodePair().first, street.second->nodePair().second, true);
    }
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::importMatrix(const std::string& fileName, bool isAdj) {
    // check the file extension
    std::string fileExt = fileName.substr(fileName.find_last_of(".") + 1);
    if (fileExt == "dsm") {
      std::ifstream file{fileName};
      if (!file.is_open()) {
        std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                             "File not found"};
        throw std::invalid_argument(errorMsg);
      }
      Id rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                             "Adjacency matrix must be square"};
        throw std::invalid_argument(errorMsg);
      }
      m_adjacency = make_shared<SparseMatrix<Id, bool>>(rows, cols);
      // each line has (should have) 3 elements
      while (!file.eof()) {
        Id index;
        bool val;
        file >> index >> val;
        m_adjacency->insert(index, val);
        const Id node1{static_cast<Id>(index / rows)};
        const Id node2{static_cast<Id>(index % cols)};
        m_nodes.insert_or_assign(node1, make_shared<Node<Id, Size>>(node1));
        m_nodes.insert_or_assign(node2, make_shared<Node<Id, Size>>(node2));
        m_streets.insert_or_assign(index,
                                   make_shared<Street<Id, Size>>(index, std::make_pair(node1, node2)));
        if (!isAdj) {
          m_streets[index]->setLength(val);
        }
      }
    } else {
      // default case: read the file as a matrix with the first two elements being the number of rows and columns and
      // the following elements being the matrix elements
      std::ifstream file{fileName};
      if (!file.is_open()) {
        std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                             "File not found"};
        throw std::invalid_argument(errorMsg);
      }
      Id rows, cols;
      file >> rows >> cols;
      if (rows != cols) {
        std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                             "Adjacency matrix must be square"};
        throw std::invalid_argument(errorMsg);
      }
      m_adjacency = make_shared<SparseMatrix<Id, bool>>(rows, cols);
      Id index{0};
      while (!file.eof()) {
        double value;
        file >> value;
        if (value < 0) {
          std::string errorMsg{"Error at line " + std::to_string(__LINE__) + " in file " + __FILE__ + ": " +
                               "Adjacency matrix elements must be positive"};
          throw std::invalid_argument(errorMsg);
        }
        if (value > 0) {
          m_adjacency->insert(index, true);
          const Id node1{static_cast<Id>(index / rows)};
          const Id node2{static_cast<Id>(index % cols)};
          m_nodes.insert_or_assign(node1, make_shared<Node<Id, Size>>(node1));
          m_nodes.insert_or_assign(node2, make_shared<Node<Id, Size>>(node2));
          m_streets.insert_or_assign(index,
                                     make_shared<Street<Id, Size>>(index, std::make_pair(node1, node2)));
          if (!isAdj) {
            m_streets[index]->setLength(value);
          }
        }
        ++index;
      }
    }
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::addNode(shared<Node<Id, Size>> node) {
    m_nodes.insert(std::make_pair(node->id(), node));
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::addNode(const Node<Id, Size>& node) {
    m_nodes.insert(std::make_pair(node.id(), make_shared<Node<Id, Size>>(node)));
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  template <typename... Tn>
    requires(is_node_v<std::remove_reference_t<Tn>> && ...)
  void Graph<Id, Size>::addNodes(Tn&&... nodes) {}

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  template <typename T1, typename... Tn>
    requires is_node_v<std::remove_reference_t<T1>> && (is_node_v<std::remove_reference_t<Tn>> && ...)
  void Graph<Id, Size>::addNodes(T1&& node, Tn&&... nodes) {
    addNode(std::forward<T1>(node));
    addNodes(std::forward<Tn>(nodes)...);
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::addStreet(shared<Street<Id, Size>> street) {
    // insert street
    m_streets.insert(std::make_pair(street->id(), street));
    // insert nodes
    const Id node1{street.nodePair().first};
    const Id node2{street.nodePair().second};
    m_nodes.insert_or_assign(node1);
    m_nodes.insert_or_assign(node2);
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  void Graph<Id, Size>::addStreet(const Street<Id, Size>& street) {
    // insert street
    m_streets.insert(std::make_pair(street.id(), make_shared<Street<Id, Size>>(street)));
    // insert nodes
    const Id node1{street.nodePair().first};
    const Id node2{street.nodePair().second};
    m_nodes.insert_or_assign(node1, make_shared<Node<Id, Size>>(node1));
    m_nodes.insert_or_assign(node2, make_shared<Node<Id, Size>>(node2));
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  template <typename T1>
    requires is_street_v<std::remove_reference_t<T1>>
  void Graph<Id, Size>::addStreets(T1&& street) {
    // insert street
    m_streets.insert(std::make_pair(street.id(), make_shared<Street<Id, Size>>(street)));
    // insert nodes
    const Id node1{street.nodePair().first};
    const Id node2{street.nodePair().second};
    m_nodes.insert_or_assign(node1, make_shared<Node<Id, Size>>(node1));
    m_nodes.insert_or_assign(node2, make_shared<Node<Id, Size>>(node2));
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  template <typename T1, typename... Tn>
    requires is_street_v<std::remove_reference_t<T1>> && (is_street_v<std::remove_reference_t<Tn>> && ...)
  void Graph<Id, Size>::addStreets(T1&& street, Tn&&... streets) {
    addStreet(std::forward<T1>(street));
    addStreets(std::forward<Tn>(streets)...);
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  shared<SparseMatrix<Id, bool>> Graph<Id, Size>::adjMatrix() const {
    return m_adjacency;
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  std::unordered_map<Id, shared<Node<Id, Size>>> Graph<Id, Size>::nodeSet() const {
    return m_nodes;
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  std::unordered_map<Id, shared<Street<Id, Size>>> Graph<Id, Size>::streetSet() const {
    return m_streets;
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  std::optional<DijkstraResult<Id>> Graph<Id, Size>::shortestPath(const Node<Id, Size>& source,
                                                                  const Node<Id, Size>& destination) const {
    return dijkstra(source.id(), destination.id());
  }

  template <typename Id, typename Size>
    requires(std::unsigned_integral<Id> && std::unsigned_integral<Size>)
  std::optional<DijkstraResult<Id>> Graph<Id, Size>::shortestPath(Id source, Id destination) const {
    std::unordered_map<Id, shared<Node<Id, Size>>> unvisitedNodes{m_nodes};
    if (!unvisitedNodes.contains(source)) {
      return std::nullopt;
    }

    const size_t n_nodes{m_nodes.size()};
    auto adj{*m_adjacency};

    std::unordered_set<Id> visitedNodes;
    std::vector<std::pair<Id, double>> dist(n_nodes);
    std::for_each(dist.begin(), dist.end(), [count = 0](auto& element) mutable -> void {
      element.first = count;
      element.second = std::numeric_limits<double>::max();
      ++count;
    });
    dist[source] = std::make_pair(source, 0.);

    std::vector<Id> prev(n_nodes);
    prev[source] = std::numeric_limits<Id>::max();
    double distance{};

    while (unvisitedNodes.size() != 0) {
      source = std::min_element(unvisitedNodes.begin(),
                                unvisitedNodes.end(),
                                [&dist](const auto& a, const auto& b) -> bool {
                                  return dist[a.first].second < dist[b.first].second;
                                })
                   ->first;
      distance = dist[source].second;
      unvisitedNodes.erase(source);
      visitedNodes.insert(source);

      // if the destination is reached, return the path
      if (source == destination) {
        std::vector<Id> path{source};
        Id previous{source};
        while (true) {
          previous = prev[previous];
          if (previous == std::numeric_limits<Id>::max()) {
            break;
          }
          path.push_back(previous);
        }
        std::reverse(path.begin(), path.end());
        return DijkstraResult<Id>(path, distance);
      }

      const auto& neighbors{adj.getRow(source)};
      // if the node is isolated, stop the algorithm
      if (neighbors.size() == 0) {
        return std::nullopt;
      }

      for (const auto& neighbour : neighbors) {
        // if the node has already been visited, skip it
        if (visitedNodes.find(neighbour.first) != visitedNodes.end()) {
          continue;
        }

        double streetLength{std::find_if(m_streets.cbegin(),
                                         m_streets.cend(),
                                         [source, &neighbour](const auto& street) -> bool {
                                           return street.second->nodePair().first == source &&
                                                  street.second->nodePair().second == neighbour.first;
                                         })
                                ->second->length()};
        // if current path is shorter than the previous one, update the distance
        if (streetLength + dist[source].second < dist[neighbour.first].second) {
          dist[neighbour.first].second = streetLength + dist[source].second;
          prev[neighbour.first] = source;
        }
      }

      adj.emptyColumn(source);
    }

    return std::nullopt;
  }
};  // namespace dsm

#endif
