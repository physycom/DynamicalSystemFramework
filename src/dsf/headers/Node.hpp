/// @file       /src/dsf/headers/Node.hpp
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

namespace dsf {
  /// @brief The Node class represents the concept of a node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Node {
  protected:
    Id m_id;
    std::optional<std::pair<double, double>> m_coords;
    std::string m_name;

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
        : m_id{other.m_id}, m_coords{other.m_coords}, m_name{other.m_name} {};
    virtual ~Node() = default;

    Node& operator=(Node const& other) {
      if (this != &other) {
        m_id = other.m_id;
        m_coords = other.m_coords;
        m_name = other.m_name;
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
    /// @brief Get the node's id
    /// @return Id The node's id
    Id id() const { return m_id; }
    /// @brief Get the node's coordinates
    /// @return std::optional<std::pair<double, double>> A std::pair containing the node's coordinates
    const std::optional<std::pair<double, double>>& coords() const { return m_coords; }
    /// @brief Get the node's name
    /// @return std::string The node's name
    const std::string& name() const { return m_name; }

    virtual bool isStation() const noexcept { return false; }
  };
};  // namespace dsf
