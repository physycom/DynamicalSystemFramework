#include "../headers/TrafficLight.hpp"

#include <format>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace dsf {
  TrafficLight& TrafficLight::operator++() {
    m_counter = (m_counter + 1) % m_cycleTime;
    return *this;
  }

  void TrafficLight::setPhases(std::vector<TrafficLightPhase> phases) {
    Delay totalGreenTime{0};
    for (auto const& phase : phases) {
      totalGreenTime += phase.greenTime();
      auto const& streetIds = phase.streetIds();
      if (streetIds.empty()) {
        throw std::invalid_argument(
            std::format("Traffic light phase with green time {} has no streets assigned.",
                        phase.greenTime()));
      }
      for (auto const& streetId : streetIds) {
        if (std::find(m_ingoingEdges.cbegin(), m_ingoingEdges.cend(), streetId) ==
            m_ingoingEdges.cend()) {
          throw std::invalid_argument(std::format(
              "Street with id {} is not ingoing edge of traffic light with id {}.",
              streetId,
              m_id));
        }
      }
      if (totalGreenTime > m_cycleTime) {
        throw std::invalid_argument(std::format(
            "Total green time of all phases ({}) is greater than the cycle time ({}).",
            totalGreenTime,
            m_cycleTime));
      }
    }
    m_phases = std::move(phases);
  }

  bool TrafficLight::isGreen(Id const streetId, Direction direction) const {
    if (m_freeTurns.contains(streetId)) {
      Direction const freeDirection{m_freeTurns.at(streetId)};
      if (freeDirection == Direction::ANY || freeDirection == direction ||
          (freeDirection == Direction::RIGHTANDSTRAIGHT &&
           (direction == Direction::RIGHT || direction == Direction::STRAIGHT)) ||
          (freeDirection == Direction::LEFTANDSTRAIGHT &&
           (direction == Direction::LEFT || direction == Direction::STRAIGHT))) {
        return true;
      }
    }
    Delay sumTime{0};
    for (auto const& phase : m_phases) {
      sumTime += phase.greenTime();
      if (m_counter >= sumTime) {
        continue;
      }
      if (phase.streetIds().contains(streetId)) {
        if (phase.direction() == Direction::ANY || phase.direction() == direction ||
            (phase.direction() == Direction::RIGHTANDSTRAIGHT &&
             (direction == Direction::RIGHT || direction == Direction::STRAIGHT)) ||
            (phase.direction() == Direction::LEFTANDSTRAIGHT &&
             (direction == Direction::LEFT || direction == Direction::STRAIGHT))) {
          return true;
        }
      }
      return false;
    }
    return false;
  }
}  // namespace dsf