/// @file src/dsf/headers/TrafficLight.hpp
/// @brief Header file for the TrafficLight class

/// @details This file contains the definition of the TrafficLightCycle and TrafficLight classes.
///          The TrafficLightCycle class represents a cycle of a traffic light, with a green time
///          and a phase. The TrafficLight class represents a traffic light intersection node in
///          the road network. It is derived from the Intersection class and has a map of cycles for each
///          street queue.

#pragma once

#include "Intersection.hpp"
#include "../utility/Typedef.hpp"

#include <format>
#include <string>
#include <unordered_set>
#include <vector>

namespace dsf {
  class TrafficLightPhase {
  private:
    Delay m_greenTime;
    Direction m_direction;
    std::unordered_set<Id> m_streetIds;

  public:
    TrafficLightPhase(Delay greenTime, Direction direction)
        : m_greenTime{greenTime}, m_direction{direction} {}

    inline void setGreenTime(Delay const greenTime) { m_greenTime = greenTime; }
    inline void setStreetIds(std::unordered_set<Id> streetIds) {
      m_streetIds = std::move(streetIds);
    }

    inline Delay greenTime() const { return m_greenTime; }
    inline Direction direction() const { return m_direction; }
    inline std::unordered_set<Id> const& streetIds() const { return m_streetIds; }
  };

  class TrafficLight final : public Intersection {
  private:
    std::vector<TrafficLightPhase> m_phases;
    std::unordered_map<Id, Direction> m_freeTurns;
    Delay m_cycleTime;  // The total time of a full cycle
    Delay m_counter;

  public:
    /// @brief Construct a new TrafficLight object
    /// @param id The node's id
    /// @param cycleTime The node's cycle time
    TrafficLight(Id id, Delay cycleTime)
        : Intersection{id}, m_cycleTime{cycleTime}, m_counter{0} {}

    TrafficLight(Id id, Delay cycleTime, std::pair<double, double> coords)
        : Intersection{id, std::move(coords)}, m_cycleTime{cycleTime}, m_counter{0} {}

    TrafficLight(RoadJunction const& node, Delay const cycleTime, Delay const counter = 0)
        : Intersection{node}, m_cycleTime{cycleTime}, m_counter{counter} {}

    ~TrafficLight() = default;

    TrafficLight& operator++();

    inline void addFreeTurn(Id const streetId, Direction const direction) {
      m_freeTurns[streetId] = direction;
    }
    /// @brief Get the traffic light's total cycle time
    /// @return Delay The traffic light's cycle time
    inline Delay cycleTime() const { return m_cycleTime; }
    /// @brief Set the traffic light's phases
    /// @param phases std::vector<TrafficLightPhase> The traffic light's phases
    void setPhases(std::vector<TrafficLightPhase> phases);
    /// @brief Get the traffic light's phases
    /// @return std::vector<TrafficLightPhase> const& The traffic light's phases
    inline std::vector<TrafficLightPhase> const& phases() const { return m_phases; }
    inline Delay counter() const { return m_counter; }
    /// @brief Returns true if all the phases are set to their default values
    bool isDefault() const;
    /// @brief Returns true if the traffic light is green for a street and a direction
    /// @param streetId Id, the street's id
    /// @param direction Direction, the direction
    /// @return true if the traffic light is green for the street and direction
    bool isGreen(Id const streetId, Direction direction) const;
    /// @brief Resets all traffic light cycles
    /// @details For more info, see @ref TrafficLightCycle::reset()
    void resetCycles();
    inline bool isTrafficLight() const noexcept { return true; }
  };
}  // namespace dsf

// template <>
// struct std::formatter<dsf::TrafficLightPhase> {
//   constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
//   template <typename FormatContext>
//   auto format(const dsf::TrafficLightPhase& phase, FormatContext&& ctx) const {
//     return std::format_to(ctx.out(),
//                           "TrafficLightPhase (green time: {} - direction: {})",
//                           phase.greenTime(),
//                           phase.direction());
//   }
// };

// template <>
// struct std::formatter<dsf::TrafficLight> {
//   constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
//   template <typename FormatContext>
//   auto format(const dsf::TrafficLight& tl, FormatContext&& ctx) const {
//     std::string strCycles;
//     for (auto const& [streetId, cycles] : tl.cycles()) {
//       std::string strStreetCycles{std::format("\tStreet {}:\n", streetId)};
//       for (auto const& [direction, cycle] : cycles) {
//         strStreetCycles += std::format(
//             "\t\t- dir {}: {}\n", dsf::directionToString.at(direction), cycle);
//       }
//       strCycles += strStreetCycles;
//     }
//     return std::format_to(
//         ctx.out(),
//         "TrafficLight \"{}\" ({}): cycle time {} - counter {}. Cycles:\n{}",
//         tl.name(),
//         tl.id(),
//         tl.cycleTime(),
//         tl.counter(),
//         strCycles);
//   }
// };
