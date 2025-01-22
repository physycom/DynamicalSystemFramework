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
#include <vector>

#include "Road.hpp"
#include "Agent.hpp"
#include "Sensors.hpp"
#include "../utility/TypeTraits/is_numeric.hpp"
#include "../utility/queue.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The Street class represents a street in the network.
  class Street : public Road {
  private:
    std::vector<dsm::queue<Size>> m_exitQueues;
    std::vector<Direction> m_laneMapping;
    std::vector<Id> m_movingAgents;

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
           double length = Road::meanVehicleLength(),
           double maxSpeed = 13.8888888889,
           int nLanes = 1,
           std::string name = std::string(),
           std::optional<int> capacity = std::nullopt,
           int transportCapacity = 1);

    /// @brief Set the street's queue
    /// @param queue The street's queue
    inline void setQueue(dsm::queue<Size> queue, size_t index) {
      m_exitQueues[index] = std::move(queue);
    }
    /// @brief Set the mean vehicle length
    /// @param meanVehicleLength The mean vehicle length
    /// @throw std::invalid_argument If the mean vehicle length is negative
    static void setMeanVehicleLength(double meanVehicleLength);

    /// @brief Get the street's waiting agents
    /// @return std::set<Id>, The street's waiting agents
    std::vector<Id> const& movingAgents() const;
    /// @brief Get the street's queue
    /// @return dsm::queue<Size>, The street's queue
    const dsm::queue<Size>& queue(size_t index) const { return m_exitQueues[index]; }
    /// @brief Get the street's queues
    /// @return std::vector<dsm::queue<Size>> The street's queues
    const std::vector<dsm::queue<Size>>& exitQueues() const { return m_exitQueues; }
    /// @brief  Get the number of agents on the street
    /// @return Size, The number of agents on the street
    int nAgents() const final;
    /// @brief Get the street's density in \f$m^{-1}\f$ or in \f$a.u.\f$, if normalized
    /// @param normalized If true, the street's density is normalized by the street's capacity
    /// @return double, The street's density
    double density(bool normalized = false) const;
    /// @brief Check if the street is full
    /// @return bool, True if the street is full, false otherwise
    bool isFull() const final { return nAgents() == m_capacity; }
    int nMovingAgents() const override { return m_movingAgents.size(); }
    /// @brief Get the number of agents on all queues
    /// @return Size The number of agents on all queues
    int nExitingAgents() const final;

    inline std::vector<Direction> const& laneMapping() const { return m_laneMapping; }

    void addAgent(Id agentId) override;
    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void enqueue(Id agentId, size_t index);
    /// @brief Remove an agent from the street's queue
    virtual std::optional<Id> dequeue(size_t index);
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    virtual bool isSpire() const { return false; };
    virtual bool isStochastic() const { return false; };
  };

  /// @brief A stochastic street is a street with a flow rate parameter
  /// @details The Stochastic Street is used to replace traffic lights with a lower level of detail.
  ///          The idea is to model the flow of agents in a street as a stochastic process, limiting
  ///          the number of agents that can exit using a parameter in [0, 1].
  ///          Thus, the flow rate parameter represents the ratio between the green time of the
  ///          traffic light and the total time of the traffic light cycle.
  class StochasticStreet : public Street {
  private:
    double m_flowRate;

  public:
    StochasticStreet(Id id, const Street& street, double flowRate);
    StochasticStreet(Id id,
                     std::pair<Id, Id> nodePair,
                     double length = Road::meanVehicleLength(),
                     double maxSpeed = 13.8888888889,
                     int nLanes = 1,
                     std::string name = std::string(),
                     double flowRate = 1.,
                     std::optional<int> capacity = std::nullopt,
                     int transportCapacity = 1);

    void setFlowRate(double const flowRate);
    double flowRate() const;

    bool isStochastic() const final;
  };

  /// @brief The SpireStreet class represents a street which is able to count agent flows in both input and output.
  /// @tparam Id The type of the street's id
  /// @tparam Size The type of the street's capacity
  class SpireStreet : public Street, public Counter {
  private:
  public:
    using Street::Street;

    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void addAgent(Id agentId) final;

    /// @brief Get the mean flow of the street
    /// @return int The flow of the street, i.e. the difference between input and output flows
    /// @details Once the flow is retrieved, bothh the input and output flows are reset to 0.
    ///     Notice that this flow is positive iff the input flow is greater than the output flow.
    int meanFlow();
    /// @brief Remove an agent from the street's queue
    /// @return std::optional<Id> The id of the agent removed from the street's queue
    std::optional<Id> dequeue(size_t index) final;
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    bool isSpire() const final { return true; };
  };

  class StochasticSpireStreet : public StochasticStreet, public Counter {
  public:
    using StochasticStreet::StochasticStreet;
    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void addAgent(Id agentId) final;

    /// @brief Get the mean flow of the street
    /// @return int The flow of the street, i.e. the difference between input and output flows
    /// @details Once the flow is retrieved, bothh the input and output flows are reset to 0.
    ///     Notice that this flow is positive iff the input flow is greater than the output flow.
    int meanFlow();
    /// @brief Remove an agent from the street's queue
    /// @return std::optional<Id> The id of the agent removed from the street's queue
    std::optional<Id> dequeue(size_t index) final;
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    bool isSpire() const final { return true; };
  };

};  // namespace dsm
