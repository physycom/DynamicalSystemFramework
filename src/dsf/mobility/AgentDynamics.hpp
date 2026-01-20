#pragma once

#include "../base/Dynamics.hpp"
#include "Agent.hpp"

#include <memory>
#include <utility>
#include <vector>
#include <tbb/concurrent_vector.h>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  template <typename NetworkType>
  class AgentDynamics : public Dynamics<NetworkType> {
  protected:
    std::vector<std::unique_ptr<Agent>> m_agents;
    tbb::concurrent_vector<std::pair<double, double>> m_travelDTs;
    std::size_t m_nAgents{0};

  public:
    AgentDynamics(NetworkType& graph, std::optional<unsigned int> seed = std::nullopt)
        : Dynamics<NetworkType>(graph, seed) {}

    /// @brief Get the agents currently in the simulation
    /// @return const tbb::concurrent_vector<std::unique_ptr<Agent>>& The agents currently in the simulation
    inline auto const& agents() const noexcept { return m_agents; }
    /// @brief Get the agents currently in the simulation
    /// @return std::size_t The number of agents currently in the simulation
    inline auto nAgents() const noexcept { return m_nAgents; }

    /// @brief Add an agent to the dynamics
    /// @param pAgent A unique pointer to the agent to add
    inline void addAgent(std::unique_ptr<Agent> pAgent) {
      spdlog::trace("Spawning agent {}", *pAgent);
      m_agents.push_back(std::move(pAgent));
      ++m_nAgents;
    }
    /// @brief Add an agent to the dynamics
    /// @tparam TArgs The types of the arguments to pass to the Agent constructor
    /// @param args The arguments to pass to the Agent constructor
    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
    inline void addAgent(TArgs&&... args) {
      addAgent(std::make_unique<Agent>(this->time_step(), std::forward<TArgs>(args)...));
    }
    /// @brief Add multiple agents to the dynamics
    /// @tparam TArgs The types of the arguments to pass to the Agent constructor
    /// @param nAgents The number of agents to add
    /// @param args The arguments to pass to the Agent constructor
    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
    inline void addAgents(std::size_t nAgents, TArgs&&... args) {
      while (nAgents-- > 0) {
        addAgent(
            std::make_unique<Agent>(this->time_step(), std::forward<TArgs>(args)...));
      }
    }

    /// @brief Remove an agent from the dynamics
    /// @param pAgent A unique pointer to the agent to remove
    /// @return A unique pointer to the removed agent
    inline auto removeAgent(std::unique_ptr<Agent> pAgent) {
      spdlog::trace("Killing agent {}", *pAgent);
      m_travelDTs.push_back(
          {pAgent->distance(),
           static_cast<double>(this->time_step() - pAgent->spawnTime())});
      --m_nAgents;
      return pAgent;
    }

    /// @brief Get the mean travel distance of the agents in \f$m\f$
    /// @param bClearData If true, clear the travel data after computing the mean
    /// @return Measurement<double> The mean travel distance and its standard deviation
    inline Measurement<double> meanTravelDistance(bool const bClearData = false) {
      if (m_travelDTs.empty()) {
        return Measurement(0., 0.);
      }
      std::vector<double> travelDistances;
      travelDistances.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        travelDistances.push_back(distance);
      }
      if (bClearData) {
        m_travelDTs.clear();
      }
      return Measurement<double>(travelDistances);
    }
    /// @brief Get the mean travel time of the agents in \f$s\f$
    /// @param bClearData If true, clear the travel data after computing the mean
    /// @return Measurement<double> The mean travel time and its standard deviation
    inline Measurement<double> meanTravelTime(bool const bClearData = false) {
      if (m_travelDTs.empty()) {
        return Measurement(0., 0.);
      }
      std::vector<double> travelTimes;
      travelTimes.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        travelTimes.push_back(time);
      }
      if (bClearData) {
        m_travelDTs.clear();
      }
      return Measurement<double>(travelTimes);
    }
    /// @brief Get the mean travel speed of the agents in \f$m/s\f$
    /// @param bClearData If true, clear the travel data after computing the mean
    /// @return Measurement<double> The mean travel speed and its standard deviation
    inline Measurement<double> meanTravelSpeed(bool const bClearData = false) {
      if (m_travelDTs.empty()) {
        return Measurement(0., 0.);
      }
      std::vector<double> travelSpeeds;
      travelSpeeds.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        if (time > 0.) {
          travelSpeeds.push_back(distance / time);
        }
      }
      if (bClearData) {
        m_travelDTs.clear();
      }
      return Measurement<double>(travelSpeeds);
    }
  };
}  // namespace dsf::mobility