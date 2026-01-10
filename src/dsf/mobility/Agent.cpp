#include "Agent.hpp"

#include <utility>

namespace dsf::mobility {
  Agent::Agent(std::time_t const& spawnTime)
      : m_spawnTime{spawnTime} {
        this->reset(spawnTime);
      }
  Agent::Agent(std::time_t const& spawnTime, Id const& sourceId, Id const& destinationId)
      : m_spawnTime{spawnTime},
        m_odPair{std::make_pair(sourceId, destinationId)} {
          this->reset(spawnTime);
        }

  void Agent::setSpeed(double const& speed) {
    if (speed < 0.) {
      throw std::invalid_argument(
          std::format("Speed ({}) must be positive", speed));
    }
    m_speed = speed;
  }
  void Agent::setFreeTime(std::time_t const& freeTime) { m_freeTime = freeTime; }

  void Agent::incrementDistance(double const& distance) {
    if (distance < 0) {
      throw std::invalid_argument(std::format(
          "Distance travelled ({}) must be positive", distance));
    }
    m_distance += distance;
  }
  void Agent::reset(std::time_t const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_speed = 0.;
    m_distance = 0.;
    m_pathIdx = 0;
  }
}  // namespace dsf::mobility