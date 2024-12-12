/// @file src/dsm/headers/Roundabout.hpp
/// @brief Header file for the Roundabout class

/// @details This file contains the definition of the Roundabout class. The Roundabout class
///          represents a roundabout node in the road network. It is derived from the Node
///          class and has a queue of agents waiting to exit the roundabout.

#pragma once

#include "Node.hpp"
#include "../utility/queue.hpp"

namespace dsm {
  /// @brief The Roundabout class represents a roundabout node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Roundabout : public Node {
  protected:
    dsm::queue<Id> m_agents;

  public:
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    explicit Roundabout(Id id) : Node{id} {};
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    Roundabout(Id id, std::pair<double, double> coords) : Node{id, coords} {};
    /// @brief Construct a new Roundabout object
    /// @param node An Intersection object
    Roundabout(const Node& node);

    virtual ~Roundabout() = default;

    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @throws std::runtime_error if the node is full
    void enqueue(Id agentId);
    /// @brief Removes the first agent from the node
    /// @return Id The agent's id
    Id dequeue();
    /// @brief Get the node's queue
    /// @return dsm::queue<Id> The node's queue
    const dsm::queue<Id>& agents() const { return m_agents; }
    /// @brief Returns the node's density
    /// @return double The node's density
    double density() const override {
      return static_cast<double>(m_agents.size()) / m_capacity;
    }
    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    bool isFull() const override { return m_agents.size() == this->m_capacity; }
    /// @brief Returns true if the node is a roundabout
    /// @return bool True if the node is a roundabout
    bool isRoundabout() const noexcept override { return true; }
  };
}  // namespace dsm