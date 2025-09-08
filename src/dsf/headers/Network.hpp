#pragma once

#include <cassert>
#include <unordered_map>

#include "AdjacencyMatrix.hpp"
#include "Edge.hpp"
#include "Node.hpp"

namespace dsf {
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  class Network {
  protected:
    AdjacencyMatrix m_adjacencyMatrix;
    std::vector<std::unique_ptr<node_t>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<edge_t>> m_edges;

    Id m_cantorPairingHashing(Id u, Id v) const;
    Id m_cantorPairingHashing(std::pair<Id, Id> const& pair) const;

  public:
    /// @brief Construct a new Network object
    /// @param adj The adjacency matrix representing the network
    explicit Network(AdjacencyMatrix const& adj);

    /// @brief Get the adjacency matrix
    /// @return AdjacencyMatrix The adjacency matrix
    AdjacencyMatrix const& adjacencyMatrix() const;
    /// @brief Get the nodes as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<node_t>> The nodes
    std::vector<std::unique_ptr<node_t>> const& nodes() const;
    /// @brief Get the edges as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<edge_t>> The edges
    std::unordered_map<Id, std::unique_ptr<edge_t>> const& edges() const;
    /// @brief Get the number of nodes
    /// @return size_t The number of nodes
    size_t nNodes() const;
    /// @brief Get the number of edges
    /// @return size_t The number of edges
    size_t nEdges() const;

    /// @brief Add a node to the network
    /// @tparam TNode The type of the node (default is node_t)
    /// @tparam TArgs The types of the arguments
    /// @param nodeId The node's id
    /// @param args The arguments to pass to the node's constructor
    template <typename TNode = node_t, typename... TArgs>
      requires(std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TNode, TArgs...>)
    Id addNode(TArgs&&... args);

    void addNDefaultNodes(size_t n);

    /// @brief Add an edge to the network
    /// @tparam TEdge The type of the edge (default is edge_t)
    /// @tparam TArgs The types of the arguments
    /// @param edgeId The edge's id
    /// @param args The arguments to pass to the edge's constructor
    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, TArgs...>)
    Id addEdge(TArgs&&... args);

    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return std::unique_ptr<node_t> const& A const reference to the node
    std::unique_ptr<node_t> const& node(Id nodeId) const;
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return std::unique_ptr<edge_t> const& A const reference to the edge
    std::unique_ptr<edge_t> const& edge(Id edgeId) const;

    std::unique_ptr<edge_t> const& edge(Id source, Id target) const;
    /// @brief Get a node by id
    /// @tparam TNode The type of the node
    /// @param nodeId The node's id
    /// @return TNode& A reference to the node
    template <typename TNode>
      requires(std::is_base_of_v<node_t, TNode>)
    TNode& node(Id nodeId);
    /// @brief Get an edge by id
    /// @tparam TEdge The type of the edge
    /// @param edgeId The edge's id
    /// @return TEdge& A reference to the edge
    template <typename TEdge>
      requires(std::is_base_of_v<edge_t, TEdge>)
    TEdge& edge(Id edgeId);
  };

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Id Network<node_t, edge_t>::m_cantorPairingHashing(Id u, Id v) const {
    return ((u + v) * (u + v + 1)) / 2 + v;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Id Network<node_t, edge_t>::m_cantorPairingHashing(std::pair<Id, Id> const& pair) const {
    return m_cantorPairingHashing(pair.first, pair.second);
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Network<node_t, edge_t>::Network(AdjacencyMatrix const& adj) {
    auto const& values{adj.elements()};
    // Add as many nodes as adj.n()
    addNDefaultNodes(adj.n());
    std::for_each(values.cbegin(), values.cend(), [&](auto const& pair) {
      addEdge(m_cantorPairingHashing(pair), std::make_pair(pair.first, pair.second));
    });
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  AdjacencyMatrix const& Network<node_t, edge_t>::adjacencyMatrix() const {
    return m_adjacencyMatrix;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<std::unique_ptr<node_t>> const& Network<node_t, edge_t>::nodes() const {
    return m_nodes;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unordered_map<Id, std::unique_ptr<edge_t>> const& Network<node_t, edge_t>::edges()
      const {
    return m_edges;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  size_t Network<node_t, edge_t>::nNodes() const {
    return m_nodes.size();
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  size_t Network<node_t, edge_t>::nEdges() const {
    return m_edges.size();
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode, typename... TArgs>
    requires(std::is_base_of_v<node_t, TNode> && std::constructible_from<TNode, TArgs...>)
  Id Network<node_t, edge_t>::addNode(TArgs&&... args) {
    m_nodes.push_back(std::make_unique<TNode>(std::forward<TArgs>(args)...));
    return static_cast<Id>(m_nodes.size() - 1);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  void Network<node_t, edge_t>::addNDefaultNodes(size_t n) {
    auto const currentSize{m_nodes.size()};
    for (size_t i = 0; i < n; ++i) {
      addNode(static_cast<Id>(currentSize + i));
    }
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge, typename... TArgs>
    requires(std::is_base_of_v<edge_t, TEdge> && std::constructible_from<TEdge, TArgs...>)
  Id Network<node_t, edge_t>::addEdge(TArgs&&... args) {
    TEdge tmpEdge(std::forward<TArgs>(args)...);
    auto const& geometry{tmpEdge.geometry()};
    Id sourceNodeId{0}, targetNodeId{0};
    auto it =
        std::find_if(m_nodes.cbegin(), m_nodes.cend(), [&tmpEdge](auto const& node) {
          return node->id() == tmpEdge.source();
        });
    if (it == m_nodes.cend()) {
      if (!geometry.empty()) {
        sourceNodeId = addNode(tmpEdge.source(), geometry.front());
      } else {
        sourceNodeId = addNode(tmpEdge.source());
      }
    } else {
      sourceNodeId = static_cast<Id>(std::distance(m_nodes.cbegin(), it));
    }
    it = std::find_if(m_nodes.cbegin(), m_nodes.cend(), [&tmpEdge](auto const& node) {
      return node->id() == tmpEdge.target();
    });
    if (it == m_nodes.cend()) {
      if (!geometry.empty()) {
        targetNodeId = addNode(tmpEdge.target(), geometry.back());
      } else {
        targetNodeId = addNode(tmpEdge.target());
      }
    } else {
      targetNodeId = static_cast<Id>(std::distance(m_nodes.cbegin(), it));
    }
    auto const& edgeId{m_cantorPairingHashing(sourceNodeId, targetNodeId)};
    if (m_edges.contains(edgeId)) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Edge with internal id {} ({}) connecting {} to {} already exists.",
                      edgeId,
                      tmpEdge.id(),
                      sourceNodeId,
                      targetNodeId)));
    }
    m_adjacencyMatrix.insert(sourceNodeId, targetNodeId);
    m_edges.emplace(std::make_pair(edgeId, std::make_unique<TEdge>(std::move(tmpEdge))));

    return edgeId;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<node_t> const& Network<node_t, edge_t>::node(Id nodeId) const {
    return m_nodes.at(nodeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id edgeId) const {
    return m_edges.at(edgeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id source,
                                                               Id target) const {
    return m_edges.at(m_cantorPairingHashing(source, target));
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode>
    requires(std::is_base_of_v<node_t, TNode>)
  TNode& Network<node_t, edge_t>::node(Id nodeId) {
    return dynamic_cast<TNode&>(*m_nodes.at(nodeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge>
    requires(std::is_base_of_v<edge_t, TEdge>)
  TEdge& Network<node_t, edge_t>::edge(Id edgeId) {
    return dynamic_cast<TEdge&>(*m_edges.at(edgeId));
  }
}  // namespace dsf