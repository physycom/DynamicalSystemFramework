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
    Id m_id;
    std::vector<Id> m_trip;
    std::optional<Id> m_streetId;
    std::optional<Id> m_srcNodeId;
    delay_t m_delay;
    double m_speed;
    double m_distance;    // Travelled distance
    unsigned int m_time;  // Travelled time
    size_t m_itineraryIdx;

  public:
    /// @brief Construct a new Agent object
    /// @param id The agent's id
    /// @param itineraryId Optional, The agent's destination node. If not provided, the agent is a random agent
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(Id id,
          std::optional<Id> itineraryId = std::nullopt,
          std::optional<Id> srcNodeId = std::nullopt);
    /// @brief Construct a new Agent object
    /// @param id The agent's id
    /// @param itineraryIds The agent's itinerary
    /// @param srcNodeId Optional, The id of the source node of the agent
    Agent(Id id, std::vector<Id> const& trip, std::optional<Id> srcNodeId = std::nullopt);
    /// @brief Set the street occupied by the agent
    /// @param streetId The id of the street currently occupied by the agent
    void setStreetId(Id streetId) { m_streetId = streetId; }
    /// @brief Set the agent's speed
    /// @param speed, The agent's speed
    /// @throw std::invalid_argument, if speed is negative
    void setSpeed(double speed);
    /// @brief Increment the agent's delay by 1
    /// @throw std::overflow_error, if delay has reached its maximum value
    void incrementDelay();
    /// @brief Increment the agent's delay by a given value
    /// @param delay The agent's delay
    /// @throw std::overflow_error, if delay has reached its maximum value
    void incrementDelay(delay_t const delay);
    /// @brief Decrement the agent's delay by 1
    /// @throw std::underflow_error, if delay has reached its minimum value
    void decrementDelay();
    /// @brief Increment the agent's distance by its speed * 1 second
    void incrementDistance() { m_distance += m_speed; }
    /// @brief Increment the agent's distance by a given value
    /// @param distance The value to increment the agent's distance byù
    /// @throw std::invalid_argument, if distance is negative
    void incrementDistance(double distance);
    /// @brief Increment the agent's time by 1
    /// @throw std::overflow_error, if time has reached its maximum value
    void incrementTime();
    /// @brief Increment the agent's time by a given value
    /// @param time The value to increment the agent's time by
    /// @throw std::overflow_error, if time has reached its maximum value
    void incrementTime(unsigned int const time);
    /// @brief Reset the agent's time to 0
    void resetTime() { m_time = 0; }
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
    void reset();

    /// @brief Get the agent's id
    /// @return The agent's id
    Id id() const { return m_id; }
    /// @brief Get the agent's itinerary
    /// @return The agent's itinerary
    Id itineraryId() const { return m_trip[m_itineraryIdx]; }
    /// @brief Get the agent's trip
    /// @return The agent's trip
    std::vector<Id> const& trip() const { return m_trip; }
    /// @brief Get the id of the street currently occupied by the agent
    /// @return The id of the street currently occupied by the agent
    std::optional<Id> streetId() const { return m_streetId; }
    /// @brief Get the id of the source node of the agent
    /// @return The id of the source node of the agent
    std::optional<Id> srcNodeId() const { return m_srcNodeId; }
    /// @brief Get the agent's speed
    /// @return The agent's speed
    double speed() const { return m_speed; }
    /// @brief Get the agent's delay
    /// @return	The agent's delay
    delay_t delay() const { return m_delay; }
    /// @brief Get the agent's travelled distance
    /// @return The agent's travelled distance
    double distance() const { return m_distance; }
    /// @brief Get the agent's travel time
    /// @return The agent's travel time
    unsigned int time() const { return m_time; }
    /// @brief Return true if the agent is a random agent
    /// @return True if the agent is a random agent, false otherwise
    bool isRandom() const { return m_trip.empty(); }
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Agent<delay_t>::Agent(Id id, std::optional<Id> itineraryId, std::optional<Id> srcNodeId)
      : m_id{id},
        m_trip{itineraryId.has_value() ? std::vector<Id>{itineraryId.value()}
                                       : std::vector<Id>{}},
        m_srcNodeId{srcNodeId},
        m_delay{0},
        m_speed{0.},
        m_distance{0.},
        m_time{0},
        m_itineraryIdx{0} {}

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Agent<delay_t>::Agent(Id id, std::vector<Id> const& trip, std::optional<Id> srcNodeId)
      : m_id{id},
        m_trip{trip},
        m_srcNodeId{srcNodeId},
        m_delay{0},
        m_speed{0.},
        m_distance{0.},
        m_time{0},
        m_itineraryIdx{0} {}

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::setSpeed(double speed) {
    if (speed < 0) {
      logger.error(std::format("Speed ({}) of agent {} must be positive", speed, m_id));
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
  void Agent<delay_t>::reset() {
    m_streetId = std::nullopt;
    m_delay = 0;
    m_speed = 0.;
    m_distance = 0.;
    m_time = 0;
    m_itineraryIdx = 0;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementDelay() {
    if (m_delay == std::numeric_limits<delay_t>::max()) {
      throw std::overflow_error(
          logger.buildExceptionMessage("delay_t has reached its maximum value"));
    }
    ++m_delay;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementDelay(delay_t const delay) {
    if (m_delay + delay < m_delay) {
      throw std::overflow_error(
          logger.buildExceptionMessage("delay_t has reached its maximum value"));
    }
    m_delay += delay;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::decrementDelay() {
    if (m_delay == 0) {
      throw std::underflow_error(
          logger.buildExceptionMessage("delay_t has reached its minimum value"));
    }
    --m_delay;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementDistance(double distance) {
    if (distance < 0) {
      logger.error(std::format(
          "Distance travelled ({}) by agent {} must be positive", distance, m_id));
    }
    m_distance += distance;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementTime() {
    if (m_time == std::numeric_limits<unsigned int>::max()) {
      throw std::overflow_error(
          logger.buildExceptionMessage("Time has reached its maximum value"));
    }
    ++m_time;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void Agent<delay_t>::incrementTime(unsigned int const time) {
    if (m_time + time < m_time) {
      throw std::overflow_error(
          logger.buildExceptionMessage("Time has reached its maximum value"));
    }
    m_time += time;
  }
};  // namespace dsm
