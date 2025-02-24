/// @file src/dsm/headers/Intersection.hpp
/// @brief Header file for the Intersection class

/// @details This file contains the definition of the Intersection class. The Intersection class
///          represents a node in the road network. It is derived from the Node class and has a
///          multimap of agents waiting to pass through the intersection. Agents are ordered by
///          their angle difference, emulating real-world precedence.

#pragma once

#include "Agent.hpp"
#include "Node.hpp"

#include <map>
#include <memory>
#include <set>

namespace dsm {
  /// @brief The Intersection class represents a node in the network.
  /// @tparam Id The type of the node's id. It must be an unsigned integral type.
  class Intersection : public Node {
  protected:
    std::multimap<int16_t, std::unique_ptr<Agent>> m_agents;
    std::set<Id>
        m_streetPriorities;  // A set containing the street ids that have priority - like main roads
    Size m_agentCounter;

  public:
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    explicit Intersection(Id id) : Node{id} {};
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    Intersection(Id id, std::pair<double, double> coords) : Node{id, coords} {};

    Intersection(Node const& node) : Node{node} {};

    virtual ~Intersection() = default;

    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    /// @throws std::runtime_error if the capacity is smaller than the current queue size
    void setCapacity(Size capacity) override;

    /// @brief Put an agent in the node
    /// @param agent A std::pair containing the agent's angle difference and id
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node.
    /// @throws std::runtime_error if the node is full
    void addAgent(double angle, std::unique_ptr<Agent> pAgent);
    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node.
    /// @throws std::runtime_error if the node is full
    void addAgent(std::unique_ptr<Agent> pAgent);
    // /// @brief Removes an agent from the node
    // /// @param agentId The agent's id
    // void removeAgent(Id agentId);
    /// @brief Set the node streets with priority
    /// @param streetPriorities A std::set containing the node's street priorities
    void setStreetPriorities(std::set<Id> streetPriorities) {
      m_streetPriorities = std::move(streetPriorities);
    }
    /// @brief Add a street to the node street priorities
    /// @param streetId The street's id
    void addStreetPriority(Id streetId) { m_streetPriorities.emplace(streetId); }
    /// @brief Returns the node's density
    /// @return double The node's density
    double density() const override {
      return static_cast<double>(m_agents.size()) / m_capacity;
    }
    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    bool isFull() const override { return m_agents.size() == this->m_capacity; }

    /// @brief Get the node's street priorities
    /// @details This function returns a std::set containing the node's street priorities.
    ///        If a street has priority, it means that the agents that are on that street
    ///        have priority over the agents that are on the other streets.
    /// @return std::set<Id> A std::set containing the node's street priorities
    virtual const std::set<Id>& streetPriorities() const { return m_streetPriorities; };
    /// @brief Get the node's agent ids
    /// @return std::set<Id> A std::set containing the node's agent ids
    const std::multimap<int16_t, std::unique_ptr<Agent>>& agents() { return m_agents; };
    /// @brief Returns the number of agents that have passed through the node
    /// @return Size The number of agents that have passed through the node
    /// @details This function returns the number of agents that have passed through the node
    ///          since the last time this function was called. It also resets the counter.
    Size agentCounter();

    bool isIntersection() const noexcept final { return true; }
  };
}  // namespace dsm