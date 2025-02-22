/// @file       /src/dsm/headers/Agent.hpp
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
#include "SparseMatrix.hpp"
#include "../utility/TypeTraits/is_numeric.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

#include <cassert>
#include <concepts>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace dsm {
  /// @brief The Agent class represents an agent in the network.
  /// @tparam delay_t, The type of the agent's delay. It must be a numeric type (see utility/TypeTraits/is_numeric.hpp).
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  class Agent {
  private:
    Time m_spawnTime, m_freeTime;
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
    /// @param id The agent's id
    /// @param itineraryId Optional, The agent's destination node. If not provided, the agent is a random agent
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(Time const& spawnTime,
          Id id,
          std::optional<Id> itineraryId = std::nullopt,
          std::optional<Id> srcNodeId = std::nullopt);
    /// @brief Construct a new Agent object
    /// @param id The agent's id
    /// @param itineraryIds The agent's itinerary
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(Time const& spawnTime,
          Id id,
          std::vector<Id> const& trip,
          std::optional<Id> srcNodeId = std::nullopt);
    /// @brief Set the street occupied by the agent
    /// @param streetId The id of the street currently occupied by the agent
    void setStreetId(Id streetId);
    /// @brief Set the id of the next street
    /// @param nextStreetId The id of the next street
    void setNextStreetId(Id nextStreetId) { m_nextStreetId = nextStreetId; }
    /// @brief Set the agent's speed
    /// @param speed, The agent's speed
    /// @throw std::invalid_argument, if speed is negative
    void setSpeed(double speed);
    /// @brief Set the agent's free time
    /// @param freeTime The agent's free time
    void setFreeTime(Time const& freeTime) { m_freeTime = freeTime; }
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
    void reset(Time const& spawnTime);

    /// @brief Get the agent's spawn time
    /// @return The agent's spawn time
    Time const& spawnTime() const { return m_spawnTime; }
    /// @brief Get the agent's free time
    /// @return The agent's free time
    Time const& freeTime() const { return m_freeTime; }
    /// @brief Get the agent's id
    /// @return The agent's id
    Id id() const { return m_id; }
    /// @brief Get the agent's itinerary
    /// @return The agent's itinerary
    Id itineraryId() const;
    /// @brief Get the agent's trip
    /// @return The agent's trip
    std::vector<Id> const& trip() const { return m_trip; }
    /// @brief Get the id of the street currently occupied by the agent
    /// @return The id of the street currently occupied by the agent
    std::optional<Id> streetId() const { return m_streetId; }
    /// @brief Get the id of the source node of the agent
    /// @return The id of the source node of the agent
    std::optional<Id> srcNodeId() const { return m_srcNodeId; }
    /// @brief Get the id of the next street
    /// @return The id of the next street
    std::optional<Id> nextStreetId() const { return m_nextStreetId; }
    /// @brief Get the agent's speed
    /// @return The agent's speed
    double speed() const { return m_speed; }
    /// @brief Get the agent's travelled distance
    /// @return The agent's travelled distance
    double distance() const { return m_distance; }
    /// @brief Return true if the agent is a random agent
    /// @return True if the agent is a random agent, false otherwise
    bool isRandom() const { return m_trip.empty(); }
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Agent<delay_t>::Agent(Time const& spawnTime,
                        Id id,
                        std::optional<Id> itineraryId,
                        std::optional<Id> srcNodeId)
      : m_spawnTime{spawnTime},
        m_freeTime{0},
        m_id{id},
        m_trip{itineraryId.has_value() ? std::vector<Id>{*itineraryId}
                                       : std::vector<Id>{}},
        m_srcNodeId{srcNodeId},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Agent<delay_t>::Agent(Time const& spawnTime,
                        Id id,
                        std::vector<Id> const& trip,
                        std::optional<Id> srcNodeId)
      : m_spawnTime{spawnTime},
        m_freeTime{spawnTime},
        m_id{id},
        m_trip{trip},
        m_srcNodeId{srcNodeId},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Id Agent<delay_t>::itineraryId() const {
    assert(m_itineraryIdx < m_trip.size());
    return m_trip[m_itineraryIdx];
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::setStreetId(Id streetId) {
    assert(m_nextStreetId.has_value() ? streetId == m_nextStreetId.value() : true);
    m_streetId = streetId;
    m_nextStreetId = std::nullopt;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::setSpeed(double speed) {
    if (speed < 0) {
      Logger::error(std::format("Speed ({}) of agent {} must be positive", speed, m_id));
    }
    m_speed = speed;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::updateItinerary() {
    if (m_itineraryIdx < m_trip.size() - 1) {
      ++m_itineraryIdx;
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::reset(Time const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_streetId = std::nullopt;
    m_speed = 0.;
    m_distance = 0.;
    m_itineraryIdx = 0;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementDistance(double distance) {
    if (distance < 0) {
      Logger::error(std::format(
          "Distance travelled ({}) by agent {} must be positive", distance, m_id));
    }
    m_distance += distance;
  }
};  // namespace dsm
