/// @file src/dsm/headers/TrafficLight.hpp
/// @brief Header file for the TrafficLight class

/// @details This file contains the definition of the TrafficLightCycle and TrafficLight classes.
///          The TrafficLightCycle class represents a cycle of a traffic light, with a green time
///          and a phase. The TrafficLight class represents a traffic light intersection node in
///          the road network. It is derived from the Intersection class and has a map of cycles for each
///          street queue.

#pragma once

#include "Intersection.hpp"

namespace dsm {
  class TrafficLightCycle {
  private:
    Delay m_greenTime;
    Delay m_phase;
    std::pair<Delay, Delay> m_defaultValues;

  public:
    /// @brief Construct a new TrafficLightCycle object
    /// @param greenTime Delay, the green time
    /// @param phase Delay, the phase
    TrafficLightCycle(Delay greenTime, Delay phase)
        : m_greenTime{greenTime},
          m_phase{phase},
          m_defaultValues{std::make_pair(m_greenTime, m_phase)} {}

    inline Delay greenTime() const { return m_greenTime; }
    inline Delay phase() const { return m_phase; }
    /// @brief Returns true if the traffic light is green
    /// @param cycleTime Delay, the total time of a red-green cycle
    /// @param counter Delay, the current counter
    /// @return true if counter < m_phase || (counter >= m_phase + m_greenTime && counter < cycleTime)
    bool isGreen(Delay const cycleTime, Delay const counter) const;
    /// @brief Reset the green and phase values to the default values
    /// @details The default values are the values that the cycle had when it was created
    void reset();
  };

  class TrafficLight : public Intersection {
  private:
    std::unordered_map<Id, std::vector<TrafficLightCycle>> m_cycles;
    Delay m_cycleTime;  // The total time of a red-green cycle
    Delay m_counter;

  public:
    /// @brief Construct a new TrafficLight object
    /// @param id The node's id
    /// @param cycleTime The node's cycle time
    TrafficLight(Id id, Delay cycleTime)
        : Intersection{id}, m_cycleTime{cycleTime}, m_counter{0} {}

    TrafficLight(Id id, Delay cycleTime, std::pair<double, double> coords)
        : Intersection{id, std::move(coords)}, m_cycleTime{cycleTime}, m_counter{0} {}

    TrafficLight(Node const& node, Delay const cycleTime, Delay const counter = 0)
        : Intersection{node}, m_cycleTime{cycleTime}, m_counter{counter} {}

    TrafficLight& operator++();
    /// @brief Get the maximum green time of every cycle
    /// @param priorityStreets bool, if true, only the priority streets are considered;
    ///        if false, only the non-priority streets are considered
    /// @return Delay The maximum green time
    /// @details The maximum green time is the maximum green time of all the cycles for
    ///          the priority streets if priorityStreets is true, or for the non-priority
    ///          streets if priorityStreets is false.
    Delay maxGreenTime(bool priorityStreets) const;
    /// @brief Get the minimum green time of every cycle
    /// @param priorityStreets bool, if true, only the priority streets are considered;
    ///        if false, only the non-priority streets are considered
    /// @return Delay The minimum green time
    /// @details The minimum green time is the minimum green time of all the cycles for
    ///          the priority streets if priorityStreets is true, or for the non-priority
    Delay minGreenTime(bool priorityStreets) const;
    /// @brief Get the traffic light's total cycle time
    /// @return Delay The traffic light's cycle time
    inline Delay cycleTime() const { return m_cycleTime; }
    /// @brief Set the cycle for a street and a direction
    /// @param streetId The street's id
    /// @param direction The direction
    /// @param cycle The traffic light cycle
    void setCycle(Id const streetId, Direction direction, TrafficLightCycle const& cycle);
    /// @brief Set the traffic light's cycles
    /// @param cycles std::unordered_map<Id, std::vector<TrafficLightCycle>> The traffic light's cycles
    inline void setCycles(std::unordered_map<Id, std::vector<TrafficLightCycle>> cycles) {
      m_cycles = std::move(cycles);
    }
    /// @brief Set the complementary cycle for a street
    /// @param streetId Id, The street's id
    /// @param existingCycle Id, The street's id associated with the existing cycle
    /// @throws std::invalid_argument if the street id does not exist
    /// @throws std::invalid_argument if the cycle does not exist
    /// @details The complementary cycle is a cycle that has as green time the total cycle time minus
    ///          the green time of the existing cycle. The phase is the total cycle time minus the
    ///          green time of the existing cycle, plus the phase of the existing cycle.
    void setComplementaryCycle(Id const streetId, Id const existingCycle);
    /// @brief Move a cycle from one street to another
    /// @param oldStreetId Id, the old street id
    /// @param newStreetId Id, the new street id
    void moveCycle(Id const oldStreetId, Id const newStreetId);
    /// @brief Increase the green times of the traffic light for priority streets and decrease the green times for non-priority streets
    /// @param delta Delay, the time to increase or decrease the green times
    void increaseGreenTimes(Delay const delta);
    /// @brief Decrease the green times of the traffic light for priority streets and increase the green times for non-priority streets
    /// @param delta Delay, the time to increase or decrease the green times
    void decreaseGreenTimes(Delay const delta);
    /// @brief Get the traffic light's cycles
    /// @return std::unordered_map<Id, std::vector<TrafficLightCycle>> const& The traffic light's cycles
    inline std::unordered_map<Id, std::vector<TrafficLightCycle>> const& cycles() const {
      return m_cycles;
    }
    /// @brief Returns true if the traffic light is green for a street and a direction
    /// @param streetId Id, the street's id
    /// @param direction Direction, the direction
    /// @return true if the traffic light is green for the street and direction
    bool isGreen(Id const streetId, Direction direction) const;
    /// @brief Resets all traffic light cycles
    /// @details For more info, see @ref TrafficLightCycle::reset()
    void resetCycles();
    inline bool isTrafficLight() const noexcept final { return true; }
  };
}  // namespace dsm