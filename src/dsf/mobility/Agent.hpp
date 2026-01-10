/// @file       /src/dsf/headers/Agent.hpp
/// @brief      Defines the Agent class.
///
/// @details    This file contains the definition of the Agent class.
///             The Agent class represents an agent in the network. It is templated by the type
///             of the agent's id and the size of agents, which must both be unsigned integrals.
///				      It is also templated by the delay_t type, which must be a numeric (see utility/TypeTraits/is_numeric.hpp)
///				      and represents the spatial or temporal (depending on the type of the template) distance
///				      between the agent and the one in front of it.

#pragma once

#include "Itinerary.hpp"
#include "../utility/Typedef.hpp"

#include <cassert>
#include <concepts>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsf::mobility {
  /// @brief The Agent class represents an agent in the network.
  class Agent {
  private:
    std::time_t m_spawnTime, m_freeTime;
    std::shared_ptr<std::vector<Id>> m_path;
    std::optional<std::pair<Id, Id>> m_odPair;
    std::size_t m_pathIdx;
    double m_speed;
    double m_distance;                     // Travelled distance
    std::optional<double> m_maxDistance;   // Maximum distance for stochastic agents
    std::optional<std::time_t> m_maxTime;  // Maximum time for stochastic agents

  public:
    /// @brief Construct a new Agent object
    /// @param spawnTime The agent's spawn time
    Agent(std::time_t const& spawnTime);

    /// @brief Construct a new Agent object
    /// @param spawnTime The agent's spawn time
    /// @param sourceId The source street id
    /// @param destinationId The destination street id
    Agent(std::time_t const& spawnTime, Id const& sourceId, Id const& destinationId);

    /// @brief Set the agent's speed
    /// @param speed, The agent's speed
    /// @throw std::invalid_argument, if speed is negative
    void setSpeed(double const& speed);
    /// @brief Set the agent's free time
    /// @param freeTime The agent's free time
    void setFreeTime(std::time_t const& freeTime);
    /// @brief Increment the agent's distance by a given value
    /// @param distance The value to increment the agent's distance byù
    /// @throw std::invalid_argument, if distance is negative
    void incrementDistance(double const& distance);
    /// @brief Update the agent's itinerary
    /// @details If possible, the agent's itinerary is updated by removing the first element
    /// from the itinerary's vector.
    inline void setMaxDistance(double const maxDistance) {
      maxDistance > 0. ? m_maxDistance = maxDistance
                       : throw std::invalid_argument(
                             "Agent::setMaxDistance: maxDistance must be positive");
    };
    /// @brief Set the agent's maximum time
    /// @param maxTime The agent's maximum time
    inline void setMaxTime(std::time_t const maxTime) { m_maxTime = maxTime; }

    /// @brief Reset the agent
    /// @details Reset the following values:
    /// - street id = std::nullopt
    /// - delay = 0
    /// - speed = 0
    /// - distance = 0
    /// - time = 0
    /// - itinerary index = 0
    void reset(std::time_t const& spawnTime);

    /// @brief Get the agent's spawn time
    /// @return The agent's spawn time
    inline std::time_t const& spawnTime() const noexcept { return m_spawnTime; };
    /// @brief Get the agent's free time
    /// @return The agent's free time
    inline std::time_t const& freeTime() const noexcept { return m_freeTime; };
    /// @brief Get the agent's maximum distance
    /// @return The agent's maximum distance, or throw std::logic_error if not set
    inline auto maxDistance() const {
      return m_maxDistance.has_value()
                 ? m_maxDistance.value()
                 : throw std::logic_error("Agent::maxDistance: maxDistance is not set");
    };
    /// @brief Get the agent's maximum time
    /// @return The agent's maximum time, or throw std::logic_error if not set
    inline auto maxTime() const {
      return m_maxTime.has_value()
                 ? m_maxTime.value()
                 : throw std::logic_error("Agent::maxTime: maxTime is not set");
    };
    /// @brief Get the agent's trip
    /// @return The agent's trip
    inline auto const& path() const noexcept { return m_path; };
    /// @brief Get the id of the source street of the agent
    /// @return The id of the source street of the agent
    inline auto const& odPair() const noexcept { return m_odPair; };

    inline auto const& streetId() const noexcept { return m_path->at(m_pathIdx); };
    /// @brief Get the id of the next street
    /// @return The id of the next street
    inline auto const& nextStreetId() const noexcept { return m_path->at(m_pathIdx + 1); };

    inline void advancePath() noexcept { ++m_pathIdx; };
    /// @brief Get the agent's speed
    /// @return The agent's speed
    inline double speed() const noexcept { return m_speed; };
    /// @brief Get the agent's travelled distance
    /// @return The agent's travelled distance
    inline double distance() const noexcept { return m_distance; };
    /// @brief Return true if the agent is a random agent
    /// @return True if the agent is a random agent, false otherwise
    inline bool isRandom() const noexcept { return m_odPair.has_value(); };

    inline bool hasArrived() const noexcept {
      if (m_pathIdx >= m_path->size()) {
        return true;
      }
      return false;
    };
    /// @brief Check if a random agent has arrived at its destination
    /// @param currentTime The current simulation time
    /// @return True if the agent has arrived (exceeded max distance or time), false otherwise
    inline bool hasArrived(std::optional<std::time_t> const& currentTime) const noexcept {
      if (!isRandom()) {
        return false;
      }
      if (currentTime.has_value() && m_maxTime.has_value()) {
        return (currentTime.value() - m_spawnTime) >= m_maxTime.value();
      }
      if (m_maxDistance.has_value()) {
        return m_distance >= m_maxDistance.value();
      }
      return false;
    };
  };
}  // namespace dsf::mobility

// Specialization of std::formatter for dsf::mobility::Agent
template <>
struct std::formatter<dsf::mobility::Agent> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::mobility::Agent& agent, FormatContext&& ctx) const {
    return std::format_to(
        ctx.out(),
        "Agent(OD ({}), speed: {:.2f} m/s, distance: {:.2f} m, spawnTime: {}, freeTime: {})",
        agent.odPair().has_value() ? std::format("{} -> {}", agent.odPair()->first, agent.odPair()->second) : "RANDOM",
        agent.speed(),
        agent.distance(),
        agent.spawnTime(),
        agent.freeTime());
  }
};
