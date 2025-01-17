/// @file       /src/dsm/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    The Node class represents the concept of a node in the network.
///             It is a virtual class that needs to be implemented by derived classes.

#pragma once

#include <functional>
#include <utility>
#include <stdexcept>
#include <optional>
#include <set>
#include <map>
#include <format>
#include <cassert>
#include <string>

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
    std::string m_name;
    Size m_capacity;
    int m_transportCapacity;

  public:
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_name{""}, m_capacity{1}, m_transportCapacity{1} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    Node(Id id, std::pair<double, double> coords)
        : m_id{id},
          m_coords{std::move(coords)},
          m_name{""},
          m_capacity{1},
          m_transportCapacity{1} {}

    Node(Node const& other)
        : m_id{other.m_id},
          m_coords{other.m_coords},
          m_name{other.m_name},
          m_capacity{other.m_capacity},
          m_transportCapacity{other.m_transportCapacity} {};
    virtual ~Node() = default;

    Node& operator=(Node const& other) {
      if (this != &other) {
        m_id = other.m_id;
        m_coords = other.m_coords;
        m_name = other.m_name;
        m_capacity = other.m_capacity;
        m_transportCapacity = other.m_transportCapacity;
      }
      return *this;
    }

    /// @brief Set the node's id
    /// @param id The node's id
    void setId(Id id) { m_id = id; }
    /// @brief Set the node's coordinates
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    void setCoords(std::pair<double, double> coords) { m_coords = std::move(coords); }
    /// @brief Set the node's name
    /// @param name The node's name
    void setName(const std::string& name) { m_name = name; }
    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    virtual void setCapacity(Size capacity) { m_capacity = capacity; }
    /// @brief Set the node's transport capacity
    /// @param capacity The node's transport capacity
    virtual void setTransportCapacity(int capacity) {
      if (capacity < 1) {
        throw std::invalid_argument(buildLog(std::format(
            "The transport capacity of a node ({}) must be greater than 0.", capacity)));
      }
      m_transportCapacity = capacity;
    }
    /// @brief Get the node's id
    /// @return Id The node's id
    Id id() const { return m_id; }
    /// @brief Get the node's coordinates
    /// @return std::optional<std::pair<double, double>> A std::pair containing the node's coordinates
    const std::optional<std::pair<double, double>>& coords() const { return m_coords; }
    /// @brief Get the node's name
    /// @return std::string The node's name
    const std::string& name() const { return m_name; }
    /// @brief Get the node's capacity
    /// @return Size The node's capacity
    Size capacity() const { return m_capacity; }
    /// @brief Get the node's transport capacity
    /// @return Size The node's transport capacity
    int transportCapacity() const { return m_transportCapacity; }

    virtual double density() const = 0;
    virtual bool isFull() const = 0;

    virtual bool isIntersection() const noexcept { return false; }
    virtual bool isTrafficLight() const noexcept { return false; }
    virtual bool isRoundabout() const noexcept { return false; }

    virtual bool isStation() const noexcept { return false; }
  };
};  // namespace dsm
