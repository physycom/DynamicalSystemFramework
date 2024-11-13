/// @file       /src/dsm/headers/Street.hpp
/// @brief      Defines the Street class.
///
/// @details    This file contains the definition of the Street class.
///             The Street class represents a street in the network. It is templated by the
///             type of the street's id and the type of the street's capacity.
///             The street's id and capacity must be unsigned integral types.

#pragma once

#include <optional>
#include <queue>
#include <type_traits>
#include <utility>
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <set>
#include <format>
#include <cassert>

#include "Agent.hpp"
#include "Node.hpp"
#include "../utility/TypeTraits/is_numeric.hpp"
#include "../utility/queue.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The Street class represents a street in the network.
  /// @tparam Id, The type of the street's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the street's capacity. It must be an unsigned integral type.
  class Street {
  private:
    std::vector<dsm::queue<Size>> m_exitQueues;
    std::set<Id> m_waitingAgents;
    std::pair<Id, Id> m_nodePair;
    double m_len;
    double m_maxSpeed;
    double m_angle;
    Id m_id;
    Size m_capacity;
    int16_t m_transportCapacity;
    int16_t m_nLanes;

  public:
    /// @brief Construct a new Street object starting from an existing street
    /// @details The new street has different id but same capacity, length, speed limit, and node pair as the
    ///          existing street.
    /// @param Street The existing street
    /// @param id The new street's id
    Street(Id id, const Street&);
    /// @brief Construct a new Street object
    /// @param id The street's id
    /// @param nodePair The street's node pair
    Street(Id id, std::pair<Id, Id> nodePair);
    /// @brief Construct a new Street object
    /// @details The default capacity is 1, the default length is 1, and the default speed limit is
    ///          50 km/h, i.e. 13.8888888889 m/s.
    /// @param id The street's id
    /// @param capacity The street's capacity
    /// @param len The street's length
    /// @param nodePair The street's node pair
    Street(Id id, Size capacity, double len, std::pair<Id, Id> nodePair);
    /// @brief Construct a new Street object
    /// @details The default speed limit is 50 km/h, i.e. 13.8888888889 m/s.
    /// @param id The street's id
    /// @param capacity The street's capacity
    /// @param len The street's length
    /// @param maxSpeed The street's speed limit
    /// @param nodePair The street's node pair
    Street(Id id, Size capacity, double len, double maxSpeed, std::pair<Id, Id> nodePair);
    /// @brief Construct a new Street object
    /// @details The default speed limit is 50 km/h, i.e. 13.8888888889 m/s.
    /// @param id The street's id
    /// @param capacity The street's capacity
    /// @param len The street's length
    /// @param lanes The street's number of lanes
    /// @param maxSpeed The street's speed limit
    /// @param nodePair The street's node pair
    Street(Id id,
           Size capacity,
           double len,
           double maxSpeed,
           std::pair<Id, Id> nodePair,
           int16_t nLanes);

    virtual ~Street() = default;

    /// @brief Set the street's id
    /// @param id The street's id
    void setId(Id id) { m_id = id; }
    /// @brief Set the street's capacity
    /// @param capacity The street's capacity
    void setCapacity(Size capacity) { m_capacity = capacity; }
    /// @brief Set the street's transport capacity
    /// @details The transport capacity is the maximum number of agents that can traverse the street
    ///          in a time step.
    /// @param capacity The street's transport capacity
    void setTransportCapacity(int16_t capacity) { m_transportCapacity = capacity; }
    /// @brief Set the street's length
    /// @param len The street's length
    /// @throw std::invalid_argument, If the length is negative
    void setLength(double len);
    /// @brief Set the street's queue
    /// @param queue The street's queue
    void setQueue(dsm::queue<Size> queue, size_t index) {
      m_exitQueues[index] = std::move(queue);
    }
    /// @brief Set the street's node pair
    /// @param node1 The source node of the street
    /// @param node2 The destination node of the street
    void setNodePair(Id node1, Id node2) { m_nodePair = std::make_pair(node1, node2); }
    /// @brief Set the street's node pair
    /// @param node1 The source node of the street
    /// @param node2 The destination node of the street
    void setNodePair(const Node& node1, const Node& node2) {
      m_nodePair = std::make_pair(node1.id(), node2.id());
    }
    /// @brief Set the street's node pair
    /// @param pair The street's node pair
    void setNodePair(std::pair<Id, Id> pair) { m_nodePair = std::move(pair); }
    /// @brief Set the street's speed limit
    /// @param speed The street's speed limit
    /// @throw std::invalid_argument, If the speed is negative
    void setMaxSpeed(double speed);
    /// @brief Set the street's angle
    /// @param srcNode The source node of the street
    /// @param dstNode The destination node of the street
    void setAngle(std::pair<double, double> srcNode, std::pair<double, double> dstNode);
    /// @brief Set the street's angle
    /// @param angle The street's angle
    /// @throw std::invalid_argument If the angle is negative or greater than 2 * pi
    void setAngle(double angle);
    /// @brief Set the street's number of lanes
    /// @param nLanes The street's number of lanes
    /// @throw std::invalid_argument If the number of lanes is 0
    void setNLanes(const int16_t nLanes);

    /// @brief Get the street's id
    /// @return Id, The street's id
    Id id() const { return m_id; }
    /// @brief Get the street's capacity
    /// @return Size, The street's capacity
    Size capacity() const { return m_capacity; }
    /// @brief Get the street's transport capacity
    /// @details The transport capacity is the maximum number of agents that can traverse the street
    ///          in a time step.
    /// @return Size, The street's transport capacity
    int16_t transportCapacity() const { return m_transportCapacity; }
    /// @brief Get the street's length
    /// @return double, The street's length
    double length() const { return m_len; }
    /// @brief Get the street's waiting agents
    /// @return std::set<Id>, The street's waiting agents
    const std::set<Id>& waitingAgents() const { return m_waitingAgents; }
    /// @brief Get the street's queue
    /// @return dsm::queue<Size>, The street's queue
    const dsm::queue<Size>& queue(size_t index) const { return m_exitQueues[index]; }
    /// @brief Get the street's queues
    /// @return std::vector<dsm::queue<Size>> The street's queues
    const std::vector<dsm::queue<Size>>& exitQueues() const { return m_exitQueues; }
    /// @brief Get the street's node pair
    /// @return std::pair<Id, Id>, The street's node pair
    const std::pair<Id, Id>& nodePair() const { return m_nodePair; }
    /// @brief  Get the number of agents on the street
    /// @return Size, The number of agents on the street
    Size nAgents() const;
    /// @brief Get the street's density in \f$m^{-1}\f$
    /// @return double, The street's density
    double density() const { return nAgents() / (m_len * m_nLanes); }
    /// @brief Get the street's normalized density
    /// @return double, The street's normalized density
    double normDensity() const { return nAgents() / static_cast<double>(m_capacity); }
    /// @brief Check if the street is full
    /// @return bool, True if the street is full, false otherwise
    bool isFull() const { return nAgents() == m_capacity; }
    /// @brief Get the street's speed limit
    /// @return double, The street's speed limit
    double maxSpeed() const { return m_maxSpeed; }
    /// @brief Get the street's angle
    /// @return double The street's angle
    double angle() const { return m_angle; }
    /// @brief Get the street's number of lanes
    /// @return int16_t The street's number of lanes
    int16_t nLanes() const { return m_nLanes; }
    /// @brief Get the number of agents on all queues
    /// @return Size The number of agents on all queues
    Size nExitingAgents() const;
    /// @brief Get the delta angle between the street and the previous street, normalized between -pi and pi
    /// @param previousStreetAngle The angle of the previous street
    /// @return double The delta angle between the street and the previous street
    double deltaAngle(double const previousStreetAngle) const;

    virtual void addAgent(Id agentId);
    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void enqueue(Id agentId, size_t index);
    /// @brief Remove an agent from the street's queue
    virtual std::optional<Id> dequeue(size_t index);
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    virtual bool isSpire() const { return false; };
  };

  /// @brief The SpireStreet class represents a street which is able to count agent flows in both input and output.
  /// @tparam Id The type of the street's id
  /// @tparam Size The type of the street's capacity
  class SpireStreet : public Street {
  private:
    Size m_agentCounterIn;
    Size m_agentCounterOut;

  public:
    /// @brief Construct a new SpireStreet object starting from an existing street
    /// @param id The street's id
    /// @param street The existing street
    SpireStreet(Id id, const Street& street);
    /// @brief Construct a new SpireStreet object
    /// @param id The street's id
    /// @param capacity The street's capacity
    /// @param len The street's length
    /// @param nodePair The street's node pair
    SpireStreet(Id id, Size capacity, double len, std::pair<Id, Id> nodePair);
    /// @brief Construct a new SpireStreet object
    /// @param id The street's id
    /// @param capacity The street's capacity
    /// @param len The street's length
    /// @param maxSpeed The street's speed limit
    /// @param nodePair The street's node pair
    SpireStreet(
        Id id, Size capacity, double len, double maxSpeed, std::pair<Id, Id> nodePair);
    ~SpireStreet() = default;

    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void addAgent(Id agentId) override;

    /// @brief Get the input counts of the street
    /// @param resetValue If true, the counter is reset to 0 together with the output counter.
    /// @return Size The input counts of the street
    Size inputCounts(bool resetValue = false);
    /// @brief Get the output counts of the street
    /// @param resetValue If true, the counter is reset to 0 together with the input counter.
    /// @return Size The output counts of the street
    Size outputCounts(bool resetValue = false);
    /// @brief Get the mean flow of the street
    /// @return int The flow of the street, i.e. the difference between input and output flows
    /// @details Once the flow is retrieved, bothh the input and output flows are reset to 0.
    ///     Notice that this flow is positive iff the input flow is greater than the output flow.
    int meanFlow();
    /// @brief Remove an agent from the street's queue
    /// @return std::optional<Id> The id of the agent removed from the street's queue
    std::optional<Id> dequeue(size_t index) override;
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    bool isSpire() const final { return true; };
  };

};  // namespace dsm
