/// @file       /src/dsf/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    The Node class represents the concept of a node in the network.
///             It is a virtual class that needs to be implemented by derived classes.

#pragma once

#include "../geometry/Point.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

#include <functional>
#include <utility>
#include <stdexcept>
#include <optional>
#include <set>
#include <map>
#include <format>
#include <cassert>
#include <string>

namespace dsf {
  /// @brief The Node class represents the concept of a node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Node {
  protected:
    Id m_id;
    std::optional<geometry::Point> m_geometry;
    std::string m_name;
    std::vector<Id> m_ingoingEdges;
    std::vector<Id> m_outgoingEdges;

  public:
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_name{""} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param point A geometry::Point containing the node's coordinates
    Node(Id id, geometry::Point point)
        : m_id{id}, m_geometry{std::move(point)}, m_name{""} {}

    Node(Node const& other)
        : m_id{other.m_id},
          m_geometry{other.m_geometry},
          m_name{other.m_name},
          m_ingoingEdges{other.m_ingoingEdges},
          m_outgoingEdges{other.m_outgoingEdges} {}
    virtual ~Node() = default;

    Node& operator=(Node const& other) {
      if (this != &other) {
        m_id = other.m_id;
        m_geometry = other.m_geometry;
        m_name = other.m_name;
        m_ingoingEdges = other.m_ingoingEdges;
        m_outgoingEdges = other.m_outgoingEdges;
      }
      return *this;
    }

    /// @brief Set the node's id
    /// @param id The node's id
    inline void setId(Id id) noexcept { m_id = id; }
    /// @brief Set the node's geometry
    /// @param point A geometry::Point containing the node's geometry
    inline void setGeometry(geometry::Point point) noexcept {
      m_geometry = std::move(point);
    }
    /// @brief Set the node's name
    /// @param name The node's name
    inline void setName(const std::string& name) noexcept { m_name = name; }
    /// @brief Add an ingoing edge to the node
    /// @param edgeId The edge's id
    /// @throws std::invalid_argument if the edge already exists in the ingoing edges
    inline void addIngoingEdge(Id edgeId) {
      if (std::find(m_ingoingEdges.cbegin(), m_ingoingEdges.cend(), edgeId) !=
          m_ingoingEdges.cend()) {
        throw std::invalid_argument(std::format(
            "Edge with id {} already exists in the incoming edges of node with id {}.",
            edgeId,
            m_id));
      }
      m_ingoingEdges.push_back(edgeId);
    }
    /// @brief Add an outgoing edge to the node
    /// @param edgeId The edge's id
    /// @throws std::invalid_argument if the edge already exists in the outgoing edges
    inline void addOutgoingEdge(Id edgeId) {
      if (std::find(m_outgoingEdges.cbegin(), m_outgoingEdges.cend(), edgeId) !=
          m_outgoingEdges.cend()) {
        throw std::invalid_argument(std::format(
            "Edge with id {} already exists in the outgoing edges of node with id {}.",
            edgeId,
            m_id));
      }
      m_outgoingEdges.push_back(edgeId);
    }

    /// @brief Get the node's id
    /// @return Id The node's id
    inline Id id() const { return m_id; }
    /// @brief Get the node's geometry
    /// @return std::optional<geometry::Point> A geometry::Point
    inline std::optional<geometry::Point> const& geometry() const noexcept {
      return m_geometry;
    }
    /// @brief Get the node's name
    /// @return std::string The node's name
    inline std::string const& name() const noexcept { return m_name; }

    inline std::vector<Id> const& ingoingEdges() const noexcept { return m_ingoingEdges; }
    inline std::vector<Id> const& outgoingEdges() const noexcept {
      return m_outgoingEdges;
    }

    virtual bool isStation() const noexcept { return false; }
  };
};  // namespace dsf
