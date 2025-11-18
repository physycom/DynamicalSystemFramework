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
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dsf::mobility {
  /// @brief The Agent class represents an agent in the network.
  class Agent {
  private:
    std::time_t m_spawnTime, m_freeTime;
    Id m_id;
    std::vector<Id> m_trip;
    std::optional<Id> m_streetId;
    std::optional<Id> m_srcNodeId;
    std::optional<Id> m_nextStreetId;
    size_t m_itineraryIdx;
    double m_speed;
    double m_distance;  // Travelled distance

  public:
    /// @brief Construct a new Agent object
    /// @param spawnTime The agent's spawn time
    /// @param itineraryId Optional, The agent's destination node. If not provided, the agent is a random agent
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(std::time_t const& spawnTime,
          std::optional<Id> itineraryId = std::nullopt,
          std::optional<Id> srcNodeId = std::nullopt);
    /// @brief Construct a new Agent object
    /// @param spawnTime The agent's spawn time
    /// @param itineraryIds The agent's itinerary
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(std::time_t const& spawnTime,
          std::vector<Id> const& trip,
          std::optional<Id> srcNodeId = std::nullopt);

    void setSrcNodeId(Id srcNodeId);
    /// @brief Set the street occupied by the agent
    /// @param streetId The id of the street currently occupied by the agent
    void setStreetId(std::optional<Id> streetId = std::nullopt);
    /// @brief Set the id of the next street
    /// @param nextStreetId The id of the next street
    void setNextStreetId(Id nextStreetId);
    /// @brief Set the agent's speed
    /// @param speed, The agent's speed
    /// @throw std::invalid_argument, if speed is negative
    void setSpeed(double speed);
    /// @brief Set the agent's free time
    /// @param freeTime The agent's free time
    void setFreeTime(std::time_t const& freeTime);
    /// @brief Increment the agent's distance by a given value
    /// @param distance The value to increment the agent's distance byù
    /// @throw std::invalid_argument, if distance is negative
    void incrementDistance(double distance);
    /// @brief Update the agent's itinerary
    /// @details If possible, the agent's itinerary is updated by removing the first element
    /// from the itinerary's vector.
    void updateItinerary();
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
    /// @brief Get the agent's id
    /// @return The agent's id
    inline Id id() const noexcept { return m_id; };
    /// @brief Get the agent's itinerary
    /// @return The agent's itinerary
    /// @throw std::logic_error if the agent is a random agent
    Id itineraryId() const;
    /// @brief Get the agent's trip
    /// @return The agent's trip
    inline std::vector<Id> const& trip() const noexcept { return m_trip; };
    /// @brief Get the id of the street currently occupied by the agent
    /// @return The id of the street currently occupied by the agent
    inline std::optional<Id> streetId() const noexcept { return m_streetId; };
    /// @brief Get the id of the source node of the agent
    /// @return The id of the source node of the agent
    inline std::optional<Id> srcNodeId() const noexcept { return m_srcNodeId; };
    /// @brief Get the id of the next street
    /// @return The id of the next street
    inline std::optional<Id> nextStreetId() const noexcept { return m_nextStreetId; };
    /// @brief Get the agent's speed
    /// @return The agent's speed
    inline double speed() const noexcept { return m_speed; };
    /// @brief Get the agent's travelled distance
    /// @return The agent's travelled distance
    inline double distance() const noexcept { return m_distance; };
    /// @brief Return true if the agent is a random agent
    /// @return True if the agent is a random agent, false otherwise
    inline bool isRandom() const noexcept { return m_trip.empty(); };
  };
};  // namespace dsf::mobility

// Specialization of std::formatter for dsf::mobility::Agent
template <>
struct std::formatter<dsf::mobility::Agent> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::mobility::Agent& agent, FormatContext&& ctx) const {
    auto const strItinerary = agent.trip().empty() ? std::string("RANDOM")
                                                   : std::to_string(agent.itineraryId());
    return std::format_to(
        ctx.out(),
        "Agent(id: {}, streetId: {}, srcNodeId: {}, nextStreetId: {}, "
        "itineraryId: {}, speed: {:.2f} m/s, distance: {:.2f} m, "
        "spawnTime: {}, freeTime: {})",
        agent.id(),
        agent.streetId().has_value() ? std::to_string(agent.streetId().value()) : "N/A",
        agent.srcNodeId().has_value() ? std::to_string(agent.srcNodeId().value()) : "N/A",
        agent.nextStreetId().has_value() ? std::to_string(agent.nextStreetId().value())
                                         : "N/A",
        strItinerary,
        agent.speed(),
        agent.distance(),
        agent.spawnTime(),
        agent.freeTime());
  }
};
