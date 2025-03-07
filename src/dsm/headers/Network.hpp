#pragma once

#include <cassert>
#include <unordered_map>

#include "AdjacencyMatrix.hpp"
#include "Edge.hpp"
#include "Node.hpp"

namespace dsm {
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  class Network {
  protected:
    AdjacencyMatrix m_adjacencyMatrix;
    std::unordered_map<Id, std::unique_ptr<node_t>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<edge_t>> m_edges;

    virtual void m_addMissingNodes(Id const edgeId);

  public:
    /// @brief Construct a new Network object
    /// @param adj The adjacency matrix representing the network
    explicit Network(AdjacencyMatrix const& adj);

    /// @brief Get the adjacency matrix
    /// @return AdjacencyMatrix The adjacency matrix
    AdjacencyMatrix const& adjacencyMatrix() const;
    /// @brief Get the nodes as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<node_t>> The nodes
    std::unordered_map<Id, std::unique_ptr<node_t>> const& nodes() const;
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
               std::constructible_from<TNode, Id, TArgs...>)
    void addNode(Id nodeId, TArgs&&... args);

    template <typename TEdge = edge_t>
      requires(std::is_base_of_v<edge_t, TEdge>)
    void addEdge(TEdge&& edge);
    /// @brief Add an edge to the network
    /// @tparam TEdge The type of the edge (default is edge_t)
    /// @tparam TArgs The types of the arguments
    /// @param edgeId The edge's id
    /// @param args The arguments to pass to the edge's constructor
    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, Id, TArgs...>)
    void addEdge(Id edgeId, TArgs&&... args);

    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return std::unique_ptr<node_t> const& A const reference to the node
    std::unique_ptr<node_t> const& node(Id nodeId) const;
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return std::unique_ptr<edge_t> const& A const reference to the edge
    std::unique_ptr<edge_t> const& edge(Id edgeId) const;
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
  void Network<node_t, edge_t>::m_addMissingNodes(Id const edgeId) {
    auto const srcNodeId{m_edges.at(edgeId)->source()};
    auto const dstNodeId{m_edges.at(edgeId)->target()};
    if (srcNodeId < m_adjacencyMatrix.n() && dstNodeId < m_adjacencyMatrix.n()) {
      if (!m_adjacencyMatrix.contains(srcNodeId, dstNodeId)) {
        m_adjacencyMatrix.insert(srcNodeId, dstNodeId);
      }
    } else {
      m_adjacencyMatrix.insert(srcNodeId, dstNodeId);
    }
    if (!m_nodes.contains(srcNodeId)) {
      m_nodes.emplace(srcNodeId, std::make_unique<node_t>(srcNodeId));
    }
    if (!m_nodes.contains(dstNodeId)) {
      m_nodes.emplace(dstNodeId, std::make_unique<node_t>(dstNodeId));
    }
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Network<node_t, edge_t>::Network(AdjacencyMatrix const& adj) : m_adjacencyMatrix{adj} {
    auto const& elements{m_adjacencyMatrix.elements()};
    auto const N{static_cast<Id>(elements.size())};
    std::for_each(elements.cbegin(), elements.cend(), [&](auto const& pair) {
      auto const srcNodeId{pair.first};
      auto const dstNodeId{pair.second};
      auto const edgeId{srcNodeId * N + dstNodeId};
      if (!m_nodes.contains(srcNodeId)) {
        m_nodes.emplace(srcNodeId, std::make_unique<node_t>(srcNodeId));
      }
      if (!m_nodes.contains(dstNodeId)) {
        m_nodes.emplace(dstNodeId, std::make_unique<node_t>(dstNodeId));
      }
      addEdge(edgeId, std::make_pair(srcNodeId, dstNodeId));
    });
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  AdjacencyMatrix const& Network<node_t, edge_t>::adjacencyMatrix() const {
    return m_adjacencyMatrix;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unordered_map<Id, std::unique_ptr<node_t>> const& Network<node_t, edge_t>::nodes()
      const {
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
    requires(std::is_base_of_v<node_t, TNode> &&
             std::constructible_from<TNode, Id, TArgs...>)
  void Network<node_t, edge_t>::addNode(Id nodeId, TArgs&&... args) {
    assert(!m_nodes.contains(nodeId));
    m_nodes.emplace(std::make_pair(
        nodeId, std::make_unique<TNode>(nodeId, std::forward<TArgs>(args)...)));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge>
    requires(std::is_base_of_v<edge_t, TEdge>)
  void Network<node_t, edge_t>::addEdge(TEdge&& edge) {
    assert(!m_edges.contains(edge.id()));
    m_edges.emplace(std::make_pair(edge.id(), std::make_unique<TEdge>(std::move(edge))));
    m_addMissingNodes(edge.id());
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge, typename... TArgs>
    requires(std::is_base_of_v<edge_t, TEdge> &&
             std::constructible_from<TEdge, Id, TArgs...>)
  void Network<node_t, edge_t>::addEdge(Id edgeId, TArgs&&... args) {
    assert(!m_edges.contains(edgeId));
    m_edges.emplace(std::make_pair(
        edgeId, std::make_unique<TEdge>(edgeId, std::forward<TArgs>(args)...)));
    m_addMissingNodes(edgeId);
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
}  // namespace dsm