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
    explicit Network(AdjacencyMatrix const& adj);

    AdjacencyMatrix const& adjacencyMatrix() const;
    std::unordered_map<Id, std::unique_ptr<node_t>> const& nodes() const;
    std::unordered_map<Id, std::unique_ptr<edge_t>> const& edges() const;

    size_t nNodes() const;
    size_t nEdges() const;

    template <typename... TArgs>
      requires(std::constructible_from<node_t, Id, TArgs...>)
    void addNode(Id nodeId, TArgs&&... args);
    template <typename TNode, typename... TArgs>
      requires(std::is_base_of_v<node_t, TNode> &&
               std::constructible_from<TNode, Id, TArgs...>)
    void addNode(Id nodeId, TArgs&&... args);
    template <typename... TArgs>
      requires(std::constructible_from<edge_t, Id, TArgs...>)
    void addEdge(Id edgeId, TArgs&&... args);
    template <typename TEdge, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, Id, TArgs...>)
    void addEdge(Id edgeId, TArgs&&... args);
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
  template <typename... TArgs>
    requires(std::constructible_from<node_t, Id, TArgs...>)
  void Network<node_t, edge_t>::addNode(Id nodeId, TArgs&&... args) {
    assert(!m_nodes.contains(nodeId));
    m_nodes.emplace(std::make_pair(
        nodeId, std::make_unique<node_t>(nodeId, std::forward<TArgs>(args)...)));
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
  template <typename... TArgs>
    requires(std::constructible_from<edge_t, Id, TArgs...>)
  void Network<node_t, edge_t>::addEdge(Id edgeId, TArgs&&... args) {
    assert(!m_edges.contains(edgeId));
    m_edges.emplace(std::make_pair(
        edgeId, std::make_unique<edge_t>(edgeId, std::forward<TArgs>(args)...)));
    m_addMissingNodes(edgeId);
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
}  // namespace dsm