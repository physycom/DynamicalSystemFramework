/// @file       /src/dsf/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    The Node class represents the concept of a node in the network.
///             It is a virtual class that needs to be implemented by derived classes.

#pragma once

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
    std::optional<std::pair<double, double>> m_coords;
    std::string m_name;
    std::vector<Id> m_ingoingEdges;
    std::vector<Id> m_outgoingEdges;

  public:
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_name{""} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    Node(Id id, std::pair<double, double> coords)
        : m_id{id}, m_coords{std::move(coords)}, m_name{""} {}

    Node(Node const& other)
        : m_id{other.m_id},
          m_coords{other.m_coords},
          m_name{other.m_name},
          m_ingoingEdges{other.m_ingoingEdges},
          m_outgoingEdges{other.m_outgoingEdges} {}
    virtual ~Node() = default;

    Node& operator=(Node const& other) {
      if (this != &other) {
        m_id = other.m_id;
        m_coords = other.m_coords;
        m_name = other.m_name;
        m_ingoingEdges = other.m_ingoingEdges;
        m_outgoingEdges = other.m_outgoingEdges;
      }
      return *this;
    }

    /// @brief Set the node's id
    /// @param id The node's id
    inline void setId(Id id) noexcept { m_id = id; }
    /// @brief Set the node's coordinates
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    inline void setCoords(std::pair<double, double> coords) noexcept {
      m_coords = std::move(coords);
    }
    /// @brief Set the node's name
    /// @param name The node's name
    inline void setName(const std::string& name) noexcept { m_name = name; }

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
    /// @brief Get the node's coordinates
    /// @return std::optional<std::pair<double, double>> A std::pair containing the node's coordinates
    inline std::optional<std::pair<double, double>> const& coords() const noexcept {
      return m_coords;
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
