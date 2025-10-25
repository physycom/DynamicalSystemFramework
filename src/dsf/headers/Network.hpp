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
    std::unordered_map<Id, std::unique_ptr<node_t>> m_nodes;
    std::unordered_map<Id, std::unique_ptr<edge_t>> m_edges;

    Id m_cantorHash(Id u, Id v) const;
    Id m_cantorHash(std::pair<Id, Id> const& idPair) const;

  public:
    /// @brief Construct a new empty Network object
    Network() = default;

    /// @brief Construct a new Network object
    /// @param adj The adjacency matrix representing the network
    explicit Network(AdjacencyMatrix const& adj);

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
               std::constructible_from<TNode, TArgs...>)
    void addNode(TArgs&&... args);

    template <typename TNode = node_t>
      requires(std::is_base_of_v<node_t, TNode>)
    void addNDefaultNodes(size_t n);

    /// @brief Add an edge to the network
    /// @tparam TEdge The type of the edge (default is edge_t)
    /// @tparam TArgs The types of the arguments
    /// @param edgeId The edge's id
    /// @param args The arguments to pass to the edge's constructor
    template <typename TEdge = edge_t, typename... TArgs>
      requires(std::is_base_of_v<edge_t, TEdge> &&
               std::constructible_from<TEdge, TArgs...>)
    void addEdge(TArgs&&... args);

    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return std::unique_ptr<node_t> const& A const reference to the node
    std::unique_ptr<node_t> const& node(Id nodeId) const;
    /// @brief Get a node by id
    /// @param nodeId The node's id
    /// @return std::unique_ptr<node_t>& A reference to the node
    std::unique_ptr<node_t>& node(Id nodeId);
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return std::unique_ptr<edge_t> const& A const reference to the edge
    std::unique_ptr<edge_t> const& edge(Id edgeId) const;
    /// @brief Get an edge by id
    /// @param edgeId The edge's id
    /// @return std::unique_ptr<edge_t>& A reference to the edge
    std::unique_ptr<edge_t>& edge(Id edgeId);

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
  Id Network<node_t, edge_t>::m_cantorHash(Id u, Id v) const {
    return ((u + v) * (u + v + 1)) / 2 + v;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Id Network<node_t, edge_t>::m_cantorHash(std::pair<Id, Id> const& idPair) const {
    return m_cantorHash(idPair.first, idPair.second);
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Network<node_t, edge_t>::Network(AdjacencyMatrix const& adj) {
    auto const& values{adj.elements()};
    // Add as many nodes as adj.n()
    addNDefaultNodes(adj.n());
    std::for_each(values.cbegin(), values.cend(), [&](auto const& pair) {
      addEdge(m_cantorHash(pair), std::make_pair(pair.first, pair.second));
    });
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
    requires(std::is_base_of_v<node_t, TNode> && std::constructible_from<TNode, TArgs...>)
  void Network<node_t, edge_t>::addNode(TArgs&&... args) {
    // Create unique_ptr directly with perfect forwarding
    auto pNode = std::make_unique<TNode>(std::forward<TArgs>(args)...);
    if (m_nodes.contains(pNode->id())) {
      throw std::invalid_argument(
          std::format("Node with id {} already exists in the network.", pNode->id()));
    }
    m_nodes[pNode->id()] = std::move(pNode);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode>
    requires(std::is_base_of_v<node_t, TNode>)
  void Network<node_t, edge_t>::addNDefaultNodes(size_t n) {
    auto const currentSize{m_nodes.size()};
    for (size_t i = 0; i < n; ++i) {
      addNode<TNode>(static_cast<Id>(currentSize + i));
    }
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge, typename... TArgs>
    requires(std::is_base_of_v<edge_t, TEdge> && std::constructible_from<TEdge, TArgs...>)
  void Network<node_t, edge_t>::addEdge(TArgs&&... args) {
    TEdge tmpEdge(std::forward<TArgs>(args)...);
    if (m_edges.contains(tmpEdge.id())) {
      throw std::invalid_argument(
          std::format("Edge with id {} already exists in the network.", tmpEdge.id()));
    }
    auto const& geometry{tmpEdge.geometry()};
    auto const& sourceNodeId = tmpEdge.source();
    auto const& targetNodeId = tmpEdge.target();

    // Check if source node exists, add if not
    if (!m_nodes.contains(sourceNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.source(), geometry.front());
      } else {
        addNode(tmpEdge.source());
      }
    }

    // Check if target node exists, add if not
    if (!m_nodes.contains(targetNodeId)) {
      if (!geometry.empty()) {
        addNode(tmpEdge.target(), geometry.back());
      } else {
        addNode(tmpEdge.target());
      }
    }

    // Get fresh references to both nodes after all potential vector reallocations
    auto const& sourceNode = node(sourceNodeId);
    auto const& targetNode = node(targetNodeId);
    sourceNode->addOutgoingEdge(tmpEdge.id());
    targetNode->addIngoingEdge(tmpEdge.id());
    if (geometry.empty()) {
      if (sourceNode->geometry().has_value() && targetNode->geometry().has_value()) {
        tmpEdge.setGeometry(
            dsf::geometry::PolyLine{*sourceNode->geometry(), *targetNode->geometry()});
      }
    }
    m_edges.emplace(tmpEdge.id(), std::make_unique<TEdge>(std::move(tmpEdge)));
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<node_t> const& Network<node_t, edge_t>::node(Id nodeId) const {
    return m_nodes.at(nodeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<node_t>& Network<node_t, edge_t>::node(Id nodeId) {
    return m_nodes.at(nodeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id edgeId) const {
    return m_edges.at(edgeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t>& Network<node_t, edge_t>::edge(Id edgeId) {
    return m_edges.at(edgeId);
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id source,
                                                               Id target) const {
    auto const it = std::find_if(
        m_edges.cbegin(), m_edges.cend(), [source, target](auto const& pair) {
          return pair.second->source() == source && pair.second->target() == target;
        });
    if (it == m_edges.cend()) {
      throw std::out_of_range(
          std::format("Edge with source {} and target {} not found.", source, target));
    }
    return it->second;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TNode>
    requires(std::is_base_of_v<node_t, TNode>)
  TNode& Network<node_t, edge_t>::node(Id nodeId) {
    return dynamic_cast<TNode&>(*node(nodeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  template <typename TEdge>
    requires(std::is_base_of_v<edge_t, TEdge>)
  TEdge& Network<node_t, edge_t>::edge(Id edgeId) {
    return dynamic_cast<TEdge&>(*edge(edgeId));
  }
}  // namespace dsf