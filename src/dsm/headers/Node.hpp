/// @file       /src/dsm/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    This file contains the definition of the Node class.
///             The Node class represents a node in the network. It is templated by the type
///             of the node's id, which must be an unsigned integral type.
///             The derived classes are:
///             - Intersection: represents an intersection node with a map of agents
///               - TrafficLight: represents a traffic light intersection node
///             - Roundabout: represents a roundabout node with a queue of agents

#pragma once

#include <functional>
#include <utility>
#include <stdexcept>
#include <optional>
#include <set>
#include <map>
#include <format>
#include <cassert>

#include "../utility/Logger.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The Node class represents the concept of a node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Node {
  protected:
    Id m_id;
    std::optional<std::pair<double, double>> m_coords;
    Size m_capacity;
    Size m_transportCapacity;

  public:
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_capacity{1}, m_transportCapacity{1} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    Node(Id id, std::pair<double, double> coords)
        : m_id{id}, m_coords{std::move(coords)}, m_capacity{1}, m_transportCapacity{1} {}

    Node(Node const& other)
        : m_id{other.m_id},
          m_coords{other.m_coords},
          m_capacity{other.m_capacity},
          m_transportCapacity{other.m_transportCapacity} {};
    virtual ~Node() = default;

    /// @brief Set the node's id
    /// @param id The node's id
    void setId(Id id) { m_id = id; }
    /// @brief Set the node's coordinates
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    void setCoords(std::pair<double, double> coords) { m_coords = std::move(coords); }
    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    virtual void setCapacity(Size capacity) { m_capacity = capacity; }
    /// @brief Set the node's transport capacity
    /// @param capacity The node's transport capacity
    virtual void setTransportCapacity(Size capacity) { m_transportCapacity = capacity; }
    /// @brief Get the node's id
    /// @return Id The node's id
    Id id() const { return m_id; }
    /// @brief Get the node's coordinates
    /// @return std::optional<std::pair<double, double>> A std::pair containing the node's coordinates
    const std::optional<std::pair<double, double>>& coords() const { return m_coords; }
    /// @brief Get the node's capacity
    /// @return Size The node's capacity
    Size capacity() const { return m_capacity; }
    /// @brief Get the node's transport capacity
    /// @return Size The node's transport capacity
    Size transportCapacity() const { return m_transportCapacity; }

    virtual double density() const = 0;
    virtual bool isFull() const = 0;

    virtual bool isIntersection() const noexcept { return false; }
    virtual bool isTrafficLight() const noexcept { return false; }
    virtual bool isRoundabout() const noexcept { return false; }
  };
};  // namespace dsm
