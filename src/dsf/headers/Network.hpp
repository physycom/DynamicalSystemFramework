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
    std::vector<Id> m_rowOffsets;
    std::vector<Id> m_columnIndices;
    std::vector<std::unique_ptr<node_t>> m_nodes;
    std::vector<std::unique_ptr<edge_t>> m_edges;
    std::unordered_map<Id, Id> m_mapNodeId;
    std::unordered_map<Id, Id> m_mapEdgeId;
    size_t m_n;

    Id m_cantorHash(Id u, Id v) const;
    Id m_cantorHash(std::pair<Id, Id> const& idPair) const;

    std::vector<const node_t*> m_inputNeighbors(Id internalNodeId) const;
    std::vector<const node_t*> m_outputNeighbors(Id internalNodeId) const;

  public:
    /// @brief Construct a new empty Network object
    Network() : m_n{0} { m_rowOffsets.push_back(0); }

    /// @brief Construct a new Network object
    /// @param adj The adjacency matrix representing the network
    explicit Network(AdjacencyMatrix const& adj);

    /// @brief Get the nodes as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<node_t>> The nodes
    std::vector<std::unique_ptr<node_t>> const& nodes() const;
    /// @brief Get the edges as an unordered map
    /// @return std::unordered_map<Id, std::unique_ptr<edge_t>> The edges
    std::vector<std::unique_ptr<edge_t>> const& edges() const;

    std::vector<const node_t*> inputNeighbors(Id nodeId) const;
    std::vector<const node_t*> outputNeighbors(Id nodeId) const;
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
  std::vector<const node_t*> Network<node_t, edge_t>::m_inputNeighbors(
      Id internalNodeId) const {
    std::vector<const node_t*> neighbors;
    // Iterate through all nodes to find which ones point to internalNodeId
    for (Id nodeIdx = 0; nodeIdx < m_rowOffsets.size() - 1; ++nodeIdx) {
      auto const rowStart = m_rowOffsets[nodeIdx];
      auto const rowEnd = m_rowOffsets[nodeIdx + 1];
      // Check if this node has an edge to internalNodeId
      for (auto csrIndex = rowStart; csrIndex < rowEnd; ++csrIndex) {
        if (m_columnIndices[csrIndex] == internalNodeId) {
          neighbors.push_back(m_nodes.at(nodeIdx).get());
          break;  // Found the edge, no need to check other edges from this node
        }
      }
    }
    return neighbors;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<const node_t*> Network<node_t, edge_t>::m_outputNeighbors(
      Id internalNodeId) const {
    std::vector<const node_t*> neighbors;
    auto const rowStart{m_rowOffsets[internalNodeId]};
    auto const rowEnd{m_rowOffsets[internalNodeId + 1]};
    for (auto csrIndex = rowStart; csrIndex < rowEnd; ++csrIndex) {
      auto const targetId{m_columnIndices[csrIndex]};
      neighbors.push_back(m_nodes.at(targetId).get());
    }
    return neighbors;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  Network<node_t, edge_t>::Network(AdjacencyMatrix const& adj) : m_n{0} {
    m_rowOffsets.push_back(0);
    auto const& values{adj.elements()};
    // Add as many nodes as adj.n()
    addNDefaultNodes(adj.n());
    std::for_each(values.cbegin(), values.cend(), [&](auto const& pair) {
      addEdge(m_cantorHash(pair), std::make_pair(pair.first, pair.second));
    });
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<std::unique_ptr<node_t>> const& Network<node_t, edge_t>::nodes() const {
    return m_nodes;
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<std::unique_ptr<edge_t>> const& Network<node_t, edge_t>::edges() const {
    return m_edges;
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<const node_t*> Network<node_t, edge_t>::inputNeighbors(Id nodeId) const {
    return m_inputNeighbors(m_mapNodeId.at(nodeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::vector<const node_t*> Network<node_t, edge_t>::outputNeighbors(Id nodeId) const {
    return m_outputNeighbors(m_mapNodeId.at(nodeId));
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
    // Extract first argument as node id
    Id nodeId = std::get<0>(std::forward_as_tuple(args...));
    m_nodes.push_back(std::make_unique<TNode>(std::forward<TArgs>(args)...));
    m_mapNodeId[nodeId] = static_cast<Id>(m_nodes.size() - 1);
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
  void Network<node_t, edge_t>::addEdge(TArgs&&... args) {
    TEdge tmpEdge(std::forward<TArgs>(args)...);
    auto const& geometry{tmpEdge.geometry()};
    auto it =
        std::find_if(m_nodes.cbegin(), m_nodes.cend(), [&tmpEdge](auto const& node) {
          return node->id() == tmpEdge.source();
        });
    if (it == m_nodes.cend()) {
      if (!geometry.empty()) {
        addNode(tmpEdge.source(), geometry.front());
      } else {
        addNode(tmpEdge.source());
      }
    }
    it = std::find_if(m_nodes.cbegin(), m_nodes.cend(), [&tmpEdge](auto const& node) {
      return node->id() == tmpEdge.target();
    });
    if (it == m_nodes.cend()) {
      if (!geometry.empty()) {
        addNode(tmpEdge.target(), geometry.back());
      } else {
        addNode(tmpEdge.target());
      }
    }
    auto const& sourceNodeId{m_mapNodeId.at(tmpEdge.source())};
    auto const& targetNodeId{m_mapNodeId.at(tmpEdge.target())};
    m_n = std::max(m_n, static_cast<size_t>(sourceNodeId + 1));
    m_n = std::max(m_n, static_cast<size_t>(targetNodeId + 1));
    while (m_rowOffsets.size() <= m_n) {
      m_rowOffsets.push_back(m_rowOffsets.back());
    }
    std::for_each(DSM_EXECUTION m_rowOffsets.begin() + sourceNodeId + 1,
                  m_rowOffsets.end(),
                  [](Id& x) { x++; });
    auto csrOffset = m_rowOffsets[sourceNodeId + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + csrOffset, targetNodeId);
    m_edges.insert(m_edges.begin() + csrOffset,
                   std::make_unique<TEdge>(std::move(tmpEdge)));

    // Update edge ID mapping
    m_mapEdgeId[m_edges[csrOffset]->id()] = csrOffset;
    // Update indices for edges that were shifted
    for (size_t i = csrOffset + 1; i < m_edges.size(); ++i) {
      m_mapEdgeId[m_edges[i]->id()] = i;
    }
  }

  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<node_t> const& Network<node_t, edge_t>::node(Id nodeId) const {
    return m_nodes.at(m_mapNodeId.at(nodeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<node_t>& Network<node_t, edge_t>::node(Id nodeId) {
    return m_nodes.at(m_mapNodeId.at(nodeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id edgeId) const {
    return m_edges.at(m_mapEdgeId.at(edgeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t>& Network<node_t, edge_t>::edge(Id edgeId) {
    return m_edges.at(m_mapEdgeId.at(edgeId));
  }
  template <typename node_t, typename edge_t>
    requires(std::is_base_of_v<Node, node_t> && std::is_base_of_v<Edge, edge_t>)
  std::unique_ptr<edge_t> const& Network<node_t, edge_t>::edge(Id source,
                                                               Id target) const {
    auto const& row{m_mapNodeId.at(source)};
    auto const& col{m_mapNodeId.at(target)};
    assert(row + 1 < m_rowOffsets.size());
    auto itFirst = m_columnIndices.begin() + m_rowOffsets[row];
    auto itLast = m_columnIndices.begin() + m_rowOffsets[row + 1];
    auto it = std::find(itFirst, itLast, col);
    if (it == itLast) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Edge from node {} to node {} not found.", source, target)));
    }
    size_t const index = m_rowOffsets[row] + std::distance(itFirst, it);
    return m_edges.at(index);
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
    auto const it = m_mapEdgeId.find(edgeId);
    if (it == m_mapEdgeId.end()) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Edge with id {} not found.", edgeId)));
    }
    return dynamic_cast<TEdge&>(*m_edges[it->second]);
  }
}  // namespace dsf