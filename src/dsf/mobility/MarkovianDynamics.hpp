/// @file       /src/dsf/mobility/MarkovianDynamics.hpp
/// @brief      Defines the MarkovianDynamics class.
///
/// @details    This file contains the definition of the MarkovianDynamics class.
///             The MarkovianDynamics class represents a simple Markovian dynamics model
///             for agent exchange between roads. Agents move from road j to road i with
///             probability proportional to sqrt(v_i/l_i * v_j/l_j), where v and l are
///             the max speed and length of each road respectively.
///             No shortest path, no itineraries, no special intersection handling.

#pragma once

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <format>
#include <numeric>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

#include <tbb/tbb.h>
#include <spdlog/spdlog.h>

#include "../base/Dynamics.hpp"
#include "Agent.hpp"
#include "MarkovianRoadNetwork.hpp"
#include "../utility/Typedef.hpp"
#include "../utility/Measurement.hpp"

namespace dsf::mobility {

  /// @brief The MarkovianDynamics class represents a Markovian exchange model on a road network.
  /// @details Agents are exchanged between adjacent roads with transition probability
  ///          proportional to sqrt(v_i/l_i * v_j/l_j) for an agent moving from road j to road i.
  class MarkovianDynamics : public Dynamics<MarkovianRoadNetwork> {
  private:
    std::vector<Id> m_edgeIndices;
    std::vector<std::unique_ptr<Agent>> m_pendingAgents;
    std::atomic<std::size_t> m_nAgents{0};
    std::atomic<std::size_t> m_nAddedAgents{0};
    tbb::concurrent_vector<std::pair<double, double>> m_travelDTs;

    /// @brief Compute the transition weight for an agent going from road j to road i
    /// @param pRoadFrom The road the agent is currently on (j)
    /// @param pRoadTo The road the agent would move to (i)
    /// @return double The unnormalized transition weight
    static double m_transitionWeight(
        std::unique_ptr<MarkovianRoad> const& pRoadFrom,
        std::unique_ptr<MarkovianRoad> const& pRoadTo);

    /// @brief Attempt to move one agent from a road to an adjacent road
    /// @param pRoad The road to evolve
    void m_evolveRoad(std::unique_ptr<MarkovianRoad> const& pRoad);

    /// @brief Place pending agents onto roads
    void m_placePendingAgents();

  public:
    /// @brief Construct a new MarkovianDynamics object
    /// @param graph The MarkovianRoadNetwork representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    MarkovianDynamics(MarkovianRoadNetwork& graph,
                      std::optional<unsigned int> seed = std::nullopt);

    /// @brief Add agents uniformly across all roads
    /// @param nAgents The number of agents to add
    /// @throws std::overflow_error If the network capacity would be exceeded
    void addAgentsUniformly(Size nAgents);

    /// @brief Add random agents with optional spawn weights
    /// @param nAgents The number of agents to add
    /// @param spawnWeights Optional per-road spawn weights
    void addRandomAgents(std::size_t nAgents,
                         std::unordered_map<Id, double> const& spawnWeights = {});

    /// @brief Evolve the simulation by one time step
    /// @param reinsert_agents If true, agents that exit the network are reinserted
    void evolve(bool reinsert_agents = false);

    /// @brief Get the number of agents currently in the simulation
    /// @return Size The number of agents
    Size nAgents() const;

    /// @brief Get the mean density of the roads in \f$m^{-1}\f$
    /// @param normalized If true, return normalized density
    /// @return Measurement<double> The mean density and standard deviation
    Measurement<double> roadMeanDensity(bool normalized = false) const;

    /// @brief Print a summary of the dynamics
    /// @param os The output stream (default is std::cout)
    void summary(std::ostream& os = std::cout) const;
  };

  inline double MarkovianDynamics::m_transitionWeight(
      std::unique_ptr<MarkovianRoad> const& pRoadFrom,
      std::unique_ptr<MarkovianRoad> const& pRoadTo) {
    // P(j -> i) ~ sqrt(v_i/l_i * v_j/l_j)
    auto const ratioFrom = pRoadFrom->maxSpeed() / pRoadFrom->length();
    auto const ratioTo = pRoadTo->maxSpeed() / pRoadTo->length();
    return std::sqrt(ratioFrom * ratioTo);
  }

  inline MarkovianDynamics::MarkovianDynamics(MarkovianRoadNetwork& graph,
                                              std::optional<unsigned int> seed)
      : Dynamics<MarkovianRoadNetwork>(graph, seed) {
    m_edgeIndices.reserve(this->graph().nEdges());
    for (auto const& [edgeId, _] : this->graph().edges()) {
      m_edgeIndices.push_back(edgeId);
    }
  }

  inline void MarkovianDynamics::m_evolveRoad(
      std::unique_ptr<MarkovianRoad> const& pRoad) {
    if (pRoad->nAgents() == 0) {
      return;
    }

    // Get the target node of this road
    auto const& targetNodeId = pRoad->target();
    auto const& pTargetNode = this->graph().node(targetNodeId);
    auto const& outgoingEdges = pTargetNode->outgoingEdges();

    if (outgoingEdges.empty()) {
      return;
    }

    // Build transition probabilities to all outgoing roads
    std::vector<std::pair<Id, double>> transitionWeights;
    double totalWeight = 0.0;

    for (auto const& outEdgeId : outgoingEdges) {
      auto const& pOutRoad = this->graph().edge(outEdgeId);
      if (pOutRoad->isFull() || pOutRoad->roadStatus() == RoadStatus::CLOSED) {
        continue;
      }
      double w = m_transitionWeight(pRoad, pOutRoad);
      // Scale by stationary weights
      w *= pOutRoad->stationaryWeight();
      transitionWeights.emplace_back(outEdgeId, w);
      totalWeight += w;
    }

    if (transitionWeights.empty() || totalWeight <= 0.0) {
      return;
    }

    // Probabilistic selection of next road
    std::uniform_real_distribution<double> uniformDist{0., totalWeight};
    auto const randValue = uniformDist(this->m_generator);
    double accumulated = 0.0;
    Id selectedRoadId = transitionWeights.back().first;

    for (auto const& [roadId, weight] : transitionWeights) {
      accumulated += weight;
      if (randValue < accumulated) {
        selectedRoadId = roadId;
        break;
      }
    }

    // Dequeue from current road and enqueue to selected road
    auto pAgent = pRoad->dequeue();
    auto const& pNextRoad = this->graph().edge(selectedRoadId);

    pAgent->setStreetId(selectedRoadId);
    pAgent->incrementDistance(pRoad->length());

    spdlog::debug(
        "Agent {} moved from road {} to road {} at time {}",
        pAgent->id(),
        pRoad->id(),
        selectedRoadId,
        this->time_step());

    pNextRoad->enqueue(std::move(pAgent));
  }

  inline void MarkovianDynamics::m_placePendingAgents() {
    if (m_pendingAgents.empty()) {
      return;
    }

    std::uniform_int_distribution<std::size_t> roadDist{
        0, m_edgeIndices.size() - 1};

    for (auto it = m_pendingAgents.begin(); it != m_pendingAgents.end();) {
      auto const idx = roadDist(this->m_generator);
      auto const& pRoad = this->graph().edge(m_edgeIndices[idx]);
      if (!pRoad->isFull()) {
        (*it)->setStreetId(pRoad->id());
        pRoad->enqueue(std::move(*it));
        it = m_pendingAgents.erase(it);
      } else {
        ++it;
      }
    }
  }

  inline void MarkovianDynamics::addAgentsUniformly(Size nAgents) {
    m_nAddedAgents += nAgents;
    if (this->nAgents() + nAgents > this->graph().capacity()) {
      throw std::overflow_error(std::format(
          "Cannot add {} agents. The network has {} agents with capacity {}.",
          nAgents,
          this->nAgents(),
          this->graph().capacity()));
    }

    std::uniform_int_distribution<std::size_t> roadDist{0, m_edgeIndices.size() - 1};

    for (Size i = 0; i < nAgents; ++i) {
      while (true) {
        auto const idx = roadDist(this->m_generator);
        auto const& pRoad = this->graph().edge(m_edgeIndices[idx]);
        if (!pRoad->isFull()) {
          auto pAgent = std::make_unique<Agent>(m_nAgents.load(), this->time_step());
          pAgent->setStreetId(pRoad->id());
          pRoad->enqueue(std::move(pAgent));
          ++m_nAgents;
          break;
        }
      }
    }
  }

  inline void MarkovianDynamics::addRandomAgents(
      std::size_t nAgents,
      std::unordered_map<Id, double> const& spawnWeights) {
    m_nAddedAgents += nAgents;

    if (spawnWeights.empty()) {
      // Uniform placement
      for (std::size_t i = 0; i < nAgents; ++i) {
        auto pAgent = std::make_unique<Agent>(m_nAgents.load(), this->time_step());
        m_pendingAgents.push_back(std::move(pAgent));
        ++m_nAgents;
      }
      m_placePendingAgents();
      return;
    }

    // Weighted placement
    double totalWeight = std::accumulate(
        spawnWeights.begin(), spawnWeights.end(), 0.,
        [](double sum, auto const& p) { return sum + p.second; });

    std::uniform_real_distribution<double> uniformDist{0., totalWeight};

    for (std::size_t i = 0; i < nAgents; ++i) {
      auto const randValue = uniformDist(this->m_generator);
      double accumulated = 0.0;
      Id selectedRoadId = spawnWeights.begin()->first;

      for (auto const& [roadId, weight] : spawnWeights) {
        accumulated += weight;
        if (randValue < accumulated) {
          selectedRoadId = roadId;
          break;
        }
      }

      auto const& pRoad = this->graph().edge(selectedRoadId);
      if (!pRoad->isFull()) {
        auto pAgent = std::make_unique<Agent>(m_nAgents.load(), this->time_step());
        pAgent->setStreetId(selectedRoadId);
        pRoad->enqueue(std::move(pAgent));
        ++m_nAgents;
      }
    }
  }

  inline void MarkovianDynamics::evolve(bool reinsert_agents) {
    spdlog::debug("MarkovianDynamics::evolve at time {}", this->time_step());

    // Collect agents to move: for each road, probabilistically dequeue one agent
    // and decide its next road.
    // We iterate over a shuffled copy of edge indices to avoid bias.
    auto shuffledIndices = m_edgeIndices;
    std::shuffle(shuffledIndices.begin(), shuffledIndices.end(), this->m_generator);

    for (auto const& edgeId : shuffledIndices) {
      auto const& pRoad = this->graph().edge(edgeId);
      m_evolveRoad(pRoad);
    }

    // Place any pending agents
    m_placePendingAgents();

    Dynamics<MarkovianRoadNetwork>::m_evolve();
  }

  inline Size MarkovianDynamics::nAgents() const {
    return m_nAgents.load();
  }

  inline Measurement<double> MarkovianDynamics::roadMeanDensity(bool normalized) const {
    if (this->graph().edges().empty()) {
      return Measurement(0., 0.);
    }
    std::vector<double> densities;
    densities.reserve(this->graph().nEdges());
    for (auto const& [_, pRoad] : this->graph().edges()) {
      densities.push_back(pRoad->density(normalized));
    }
    return Measurement<double>(densities);
  }

  inline void MarkovianDynamics::summary(std::ostream& os) const {
    os << "MarkovianDynamics Summary:\n";
    this->graph().describe(os);
    os << "\nNumber of added agents: " << m_nAddedAgents << '\n'
       << "Current number of agents: " << this->nAgents() << '\n';
  }

}  // namespace dsf::mobility

