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
#include <string>

#include "Edge.hpp"
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
  class Street : public Edge {
  private:
    std::vector<dsm::queue<Size>> m_exitQueues;
    std::vector<Direction> m_laneMapping;
    std::set<Id> m_waitingAgents;
    double m_length;
    double m_maxSpeed;
    double m_angle;
    std::string m_name;
    int m_nLanes;
    static double m_meanVehicleLength;

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
    /// @param length The street's length, in meters (default is the mean vehicle length)
    /// @param nLanes The street's number of lanes (default is 1)
    /// @param maxSpeed The street's speed limit, in m/s (default is 50 km/h)
    /// @param name The street's name (default is an empty string)
    /// @param capacity The street's capacity (default is the maximum number of vehicles that can fit in the street)
    /// @param transportCapacity The street's transport capacity (default is 1)
    Street(Id id,
           std::pair<Id, Id> nodePair,
           double length = m_meanVehicleLength,
           double maxSpeed = 13.8888888889,
           int nLanes = 1,
           std::string name = std::string(),
           std::optional<int> capacity = std::nullopt,
           int transportCapacity = 1);

    /// @brief Set the street's length
    /// @param len The street's length
    /// @throw std::invalid_argument, If the length is negative
    void setLength(double len);
    /// @brief Set the street's queue
    /// @param queue The street's queue
    inline void setQueue(dsm::queue<Size> queue, size_t index) {
      m_exitQueues[index] = std::move(queue);
    }
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
    /// @brief Set the mean vehicle length
    /// @param meanVehicleLength The mean vehicle length
    /// @throw std::invalid_argument If the mean vehicle length is negative
    static void setMeanVehicleLength(double meanVehicleLength);

    /// @brief Get the street's length
    /// @return double, The street's length
    double length() const { return m_length; }
    /// @brief Get the street's waiting agents
    /// @return std::set<Id>, The street's waiting agents
    const std::set<Id>& waitingAgents() const { return m_waitingAgents; }
    /// @brief Get the street's queue
    /// @return dsm::queue<Size>, The street's queue
    const dsm::queue<Size>& queue(size_t index) const { return m_exitQueues[index]; }
    /// @brief Get the street's queues
    /// @return std::vector<dsm::queue<Size>> The street's queues
    const std::vector<dsm::queue<Size>>& exitQueues() const { return m_exitQueues; }
    /// @brief  Get the number of agents on the street
    /// @return Size, The number of agents on the street
    Size nAgents() const;
    /// @brief Get the street's density in \f$m^{-1}\f$ or in \f$a.u.\f$, if normalized
    /// @param normalized If true, the street's density is normalized by the street's capacity
    /// @return double, The street's density
    double density(bool normalized = false) const;
    /// @brief Check if the street is full
    /// @return bool, True if the street is full, false otherwise
    bool isFull() const final { return nAgents() == m_capacity; }
    /// @brief Get the street's speed limit
    /// @return double, The street's speed limit
    double maxSpeed() const { return m_maxSpeed; }
    /// @brief Get the street's angle
    /// @return double The street's angle
    double angle() const { return m_angle; }
    /// @brief Get the street's number of lanes
    /// @return int16_t The street's number of lanes
    int16_t nLanes() const { return m_nLanes; }
    /// @brief Get the street's name
    /// @return std::string_view The street's name
    std::string_view name() const { return m_name; }
    /// @brief Get the number of agents on all queues
    /// @return Size The number of agents on all queues
    Size nExitingAgents() const;
    /// @brief Get the delta angle between the street and the previous street, normalized between -pi and pi
    /// @param previousStreetAngle The angle of the previous street
    /// @return double The delta angle between the street and the previous street
    double deltaAngle(double const previousStreetAngle) const;

    inline std::vector<Direction> const& laneMapping() const { return m_laneMapping; }

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
    Size m_agentCounterIn = 0;
    Size m_agentCounterOut = 0;

  public:
    using Street::Street;
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
