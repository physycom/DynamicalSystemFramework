/// @file       /src/dsf/headers/Street.hpp
/// @brief      Defines the Street class.
///
/// @details    This file contains the definition of the Street class.
///             The Street class represents a street in the network. It is templated by the
///             type of the street's id and the type of the street's capacity.
///             The street's id and capacity must be unsigned integral types.

#pragma once

#include "Agent.hpp"
#include "Road.hpp"
#include "Sensors.hpp"
#include "../utility/TypeTraits/is_numeric.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

#include <optional>
#include <queue>
#include <type_traits>
#include <utility>
#include <stdexcept>
#include <cmath>
#include <numbers>
#include <format>
#include <cassert>
#include <string>
#include <vector>

#include <fmt/format.h>

namespace dsf {

  class AgentComparator {
  public:
    template <typename T>
    bool operator()(T const& lhs, T const& rhs) const {
      return lhs->freeTime() > rhs->freeTime();
    }
  };

  class Agent;

  /// @brief The Street class represents a street in the network.
  class Street : public Road {
  private:
    std::vector<dsf::queue<std::unique_ptr<Agent>>> m_exitQueues;
    dsf::priority_queue<std::unique_ptr<Agent>,
                        std::vector<std::unique_ptr<Agent>>,
                        AgentComparator>
        m_movingAgents;
    std::vector<Direction> m_laneMapping;

  public:
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
           std::vector<std::pair<double, double>> geometry = {},
           std::optional<int> capacity = std::nullopt,
           double transportCapacity = 1.);
    Street(Street&&) = default;
    Street(Street const&) = delete;
    bool operator==(Street const& other) const;

    /// @brief Set the street's lane mapping
    /// @param laneMapping The street's lane mapping
    void setLaneMapping(std::vector<Direction> const& laneMapping);
    /// @brief Set the street's queue
    /// @param queue The street's queue
    void setQueue(dsf::queue<std::unique_ptr<Agent>> queue, size_t index);
    /// @brief Set the mean vehicle length
    /// @param meanVehicleLength The mean vehicle length
    /// @throw std::invalid_argument If the mean vehicle length is negative
    static void setMeanVehicleLength(double meanVehicleLength);

    /// @brief Get the street's queue
    /// @return dsf::queue<Size>, The street's queue
    const dsf::queue<std::unique_ptr<Agent>>& queue(size_t const& index) const {
      return m_exitQueues[index];
    }
    /// @brief Get the street's queues
    /// @return std::vector<dsf::queue<Size>> The street's queues
    std::vector<dsf::queue<std::unique_ptr<Agent>>> const& exitQueues() const {
      return m_exitQueues;
    }
    /// @brief  Get the number of agents on the street
    /// @return Size, The number of agents on the street
    int nAgents() const final;
    /// @brief Get the street's density in \f$m^{-1}\f$ or in \f$a.u.\f$, if normalized
    /// @param normalized If true, the street's density is normalized by the street's capacity
    /// @return double, The street's density
    double density(bool normalized = false) const final;
    /// @brief Check if the street is full
    /// @return bool, True if the street is full, false otherwise
    inline bool isFull() const final { return this->nAgents() == this->m_capacity; }

    dsf::priority_queue<std::unique_ptr<Agent>,
                        std::vector<std::unique_ptr<Agent>>,
                        AgentComparator>&
    movingAgents() {
      return m_movingAgents;
    }
    /// @brief Get the number of of moving agents, i.e. agents not yet enqueued
    /// @return int The number of moving agents
    int nMovingAgents() const override;
    /// @brief Get the number of agents on all queues for a given direction
    /// @param direction The direction of the agents (default is ANY)
    /// @param normalizeOnNLanes If true, the number of agents is normalized by the number of lanes
    /// @return double The number of agents on all queues for a given direction
    double nExitingAgents(Direction direction = Direction::ANY,
                          bool normalizeOnNLanes = false) const final;

    inline std::vector<Direction> const& laneMapping() const { return m_laneMapping; }

    virtual void addAgent(std::unique_ptr<Agent> pAgent);
    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void enqueue(size_t const& queueId);
    /// @brief Remove an agent from the street's queue
    /// @return Id The id of the agent removed from the street's queue
    virtual std::unique_ptr<Agent> dequeue(size_t index);
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
    StochasticStreet(Street&&, double flowRate);
    StochasticStreet(Id id,
                     std::pair<Id, Id> nodePair,
                     double length = Road::meanVehicleLength(),
                     double maxSpeed = 13.8888888889,
                     int nLanes = 1,
                     std::string name = std::string(),
                     std::vector<std::pair<double, double>> geometry = {},
                     double flowRate = 1.,
                     std::optional<int> capacity = std::nullopt,
                     double transportCapacity = 1.);

    void setFlowRate(double const flowRate);
    double flowRate() const;

    bool isStochastic() const final;
  };

  /// @brief The SpireStreet class represents a street which is able to count agent flows in both input and output.
  /// @tparam Id The type of the street's id
  /// @tparam Size The type of the street's capacity
  class SpireStreet : public Street, public Counter {
  public:
    using Street::Street;
    SpireStreet(Street&& street) : Street(std::move(street)) {}
    SpireStreet(SpireStreet&&) = default;
    SpireStreet(SpireStreet const&) = delete;
    ~SpireStreet() = default;

    /// @brief Add an agent to the street's queue
    /// @param agentId The id of the agent to add to the street's queue
    /// @throw std::runtime_error If the street's queue is full
    void addAgent(std::unique_ptr<Agent> pAgent) final;

    /// @brief Get the mean flow of the street
    /// @return int The flow of the street, i.e. the difference between input and output flows
    /// @details Once the flow is retrieved, bothh the input and output flows are reset to 0.
    ///     Notice that this flow is positive iff the input flow is greater than the output flow.
    int meanFlow();
    /// @brief Remove an agent from the street's queue
    /// @return Id The id of the agent removed from the street's queue
    std::unique_ptr<Agent> dequeue(size_t index) final;
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
    void addAgent(std::unique_ptr<Agent> pAgent) final;

    /// @brief Get the mean flow of the street
    /// @return int The flow of the street, i.e. the difference between input and output flows
    /// @details Once the flow is retrieved, bothh the input and output flows are reset to 0.
    ///     Notice that this flow is positive iff the input flow is greater than the output flow.
    int meanFlow();
    /// @brief Remove an agent from the street's queue
    /// @return std::optional<Id> The id of the agent removed from the street's queue
    std::unique_ptr<Agent> dequeue(size_t index) final;
    /// @brief Check if the street is a spire
    /// @return bool True if the street is a spire, false otherwise
    bool isSpire() const final { return true; };
  };

};  // namespace dsf

// Specialization of std::formatter for dsf::Street
template <>
struct std::formatter<dsf::Street> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::Street& street, FormatContext&& ctx) const {
    auto const& name =
        street.name().empty() ? std::string() : std::format(" \"{}\"", street.name());
    return std::format_to(ctx.out(),
                          "Street(id: {}{}, from {} to {}, length: {} m, max speed: "
                          "{:.2f} m/s, lanes: {}, agents: {}, n enqueued: {})",
                          street.id(),
                          name,
                          street.nodePair().first,
                          street.nodePair().second,
                          street.length(),
                          street.maxSpeed(),
                          street.nLanes(),
                          street.nAgents(),
                          street.nExitingAgents());
  }
};
// Specialization of fmt::formatter for dsf::Street
template <>
struct fmt::formatter<dsf::Street> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::Street& street, FormatContext& ctx) const {
    auto const& name =
        street.name().empty() ? std::string() : fmt::format(" \"{}\"", street.name());
    return fmt::format_to(ctx.out(),
                          "Street(id: {}{}, from {} to {}, length: {} m, max speed: "
                          "{:.2f} m/s, lanes: {}, agents: {}, n enqueued: {})",
                          street.id(),
                          name,
                          street.nodePair().first,
                          street.nodePair().second,
                          street.length(),
                          street.maxSpeed(),
                          street.nLanes(),
                          street.nAgents(),
                          street.nExitingAgents());
  }
};