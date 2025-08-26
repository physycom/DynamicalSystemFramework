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

namespace dsf {
  class TrafficLightCycle {
  private:
    std::set<std::pair<Id, Direction>> m_associations;
    std::pair<Delay, Delay> m_values;
    std::pair<Delay, Delay> m_defaultValues;
    double m_counts;

  public:
    /// @brief Construct a new TrafficLightCycle object
    /// @param associations std::set<std::pair<Id, Direction>>, the associations
    /// @param greenTime Delay, the green time
    /// @param phase Delay, the phase
    TrafficLightCycle(std::set<std::pair<Id, Direction>> associations,
                      Delay greenTime,
                      Delay phase)
        : m_associations{std::move(associations)},
          m_values{std::make_pair(greenTime, phase)},
          m_defaultValues{std::make_pair(greenTime, phase)},
          m_counts{0.} {}
    /// @brief Construct a new TrafficLightCycle object
    /// @param greenTime Delay, the green time
    /// @param phase Delay, the phase
    TrafficLightCycle(Delay greenTime, Delay phase)
        : m_associations{std::set<std::pair<Id, Direction>>()},
          m_values{std::make_pair(greenTime, phase)},
          m_defaultValues{std::make_pair(greenTime, phase)},
          m_counts{0.} {}

    inline void increaseCounts(double amount) { m_counts += amount; }

    inline std::set<std::pair<Id, Direction>> const& associations() const {
      return m_associations;
    }
    inline Delay greenTime() const { return m_values.first; }
    inline Delay phase() const { return m_values.second; }
    inline double counts() const { return m_counts; }
    /// @brief Returns true if the traffic light is green
    /// @param cycleTime Delay, the total time of a red-green cycle
    /// @param counter Delay, the current counter
    /// @return true if counter < m_phase || (counter >= m_phase + m_greenTime && counter < cycleTime)
    bool isGreen(Delay const cycleTime, Delay const counter) const;
    /// @brief Reset the green and phase values to the default values
    /// @details The default values are the values that the cycle had when it was created
    void reset();

    inline void clearCounts() { m_counts = 0.; }
  };

  class TrafficLight final : public Intersection {
  private:
    std::vector<TrafficLightCycle> m_cycles;
    std::vector<TrafficLightCycle> m_defaultCycles;
    Delay m_cycleTime;  // The total time of a red-green cycle
    Delay m_counter;
    static bool m_allowFreeTurns;

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

    static void setAllowFreeTurns(bool allow);

    /// @brief Get the traffic light's total cycle time
    /// @return Delay The traffic light's cycle time
    inline Delay cycleTime() const { return m_cycleTime; }

    inline void addCycle(TrafficLightCycle const& cycle) { m_cycles.push_back(cycle); }
    /// @brief Set the traffic light's cycles
    /// @param cycles std::vector<TrafficLightCycle> The traffic light's cycles
    inline void setCycles(std::vector<TrafficLightCycle> cycles) {
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
    // void setComplementaryCycle(Id const streetId, Id const existingCycle);
    /// @brief Move a cycle from one street to another
    /// @param oldStreetId Id, the old street id
    /// @param newStreetId Id, the new street id
    void moveCycle(Id const oldStreetId, Id const newStreetId);
    /// @brief Get the traffic light's cycles
    /// @return std::vector<TrafficLightCycle>& The traffic light's cycles
    inline std::vector<TrafficLightCycle>& cycles() { return m_cycles; }
    inline Delay counter() const { return m_counter; }
    /// @brief Returns true if all the cycles are set to their default values
    bool isDefault() const;
    /// @brief Returns true if the traffic light is green for a street and a direction
    /// @param streetId Id, the street's id
    /// @param direction Direction, the direction
    /// @return true if the traffic light is green for the street and direction
    bool isGreen(Id const streetId, Direction direction) const;
    /// @brief Resets all traffic light cycles
    /// @details For more info, see @ref TrafficLightCycle::reset()
    void resetCycles();
    /// @brief Clears all traffic light cycles
    inline void clearCycles() { m_cycles.clear(); }

    inline bool isTrafficLight() const noexcept { return true; }
  };
}  // namespace dsf

template <>
struct std::formatter<dsf::TrafficLightCycle> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::TrafficLightCycle& cycle, FormatContext&& ctx) const {
    return std::format_to(ctx.out(),
                          "TrafficLightCycle (green time: {} - phase shift: {})",
                          cycle.greenTime(),
                          cycle.phase());
  }
};

template <>
struct std::formatter<dsf::TrafficLight> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::TrafficLight& tl, FormatContext&& ctx) const {
    // std::string strCycles;
    // for (auto const& [streetId, cycles] : tl.cycles()) {
    //   std::string strStreetCycles{std::format("\tStreet {}:\n", streetId)};
    //   for (auto const& [direction, cycle] : cycles) {
    //     strStreetCycles += std::format(
    //         "\t\t- dir {}: {}\n", dsf::directionToString.at(direction), cycle);
    //   }
    //   strCycles += strStreetCycles;
    // }
    // return std::format_to(
    //     ctx.out(),
    //     "TrafficLight \"{}\" ({}): cycle time {} - counter {}. Cycles:\n{}",
    //     tl.name(),
    //     tl.id(),
    //     tl.cycleTime(),
    //     tl.counter(),
    //     strCycles);
    return std::format_to(ctx.out(),
                          "");
  }
};
