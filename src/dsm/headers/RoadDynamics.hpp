/// @file       /src/dsm/headers/RoadDynamics.hpp
/// @brief      Defines the RoadDynamics class.
///
/// @details    This file contains the definition of the RoadDynamics class.
///             The RoadDynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include <algorithm>
#include <cassert>
#include <concepts>
#include <vector>
#include <random>
#include <span>
#include <numeric>
#include <unordered_map>
#include <cmath>
#include <cassert>
#include <format>
#include <thread>
#include <exception>
#include <fstream>
#include <iomanip>

#include "Dynamics.hpp"
#include "Agent.hpp"
#include "DijkstraWeights.hpp"
#include "Itinerary.hpp"
#include "Graph.hpp"
#include "SparseMatrix.hpp"
#include "../utility/TypeTraits/is_agent.hpp"
#include "../utility/TypeTraits/is_itinerary.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The RoadDynamics class represents the dynamics of the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  class RoadDynamics : public Dynamics<Agent<delay_t>> {
  private:
    AdjacencyMatrix m_adjMatrix_T;

  protected:
    Time m_previousOptimizationTime;

  private:
    std::optional<double> m_errorProbability;
    std::optional<double> m_passageProbability;

  protected:
    std::vector<std::pair<double, double>> m_travelDTs;
    bool m_forcePriorities;
    std::optional<delay_t> m_dataUpdatePeriod;
    std::unordered_map<Id, std::array<unsigned long long, 4>> m_turnCounts;
    std::unordered_map<Id, std::array<long, 4>> m_turnMapping;
    std::unordered_map<Id, double> m_streetTails;

    /// @brief Get the next street id
    /// @param agentId The id of the agent
    /// @param NodeId The id of the node
    /// @param streetId The id of the incoming street
    /// @return Id The id of the randomly selected next street
    virtual Id m_nextStreetId(Id agentId,
                              Id NodeId,
                              std::optional<Id> streetId = std::nullopt);
    /// @brief Increase the turn counts
    virtual void m_increaseTurnCounts(Id streetId, double delta);
    /// @brief Evolve a street
    /// @param pStreet A std::unique_ptr to the street
    /// @param reinsert_agents If true, the agents are reinserted in the simulation after they reach their destination
    /// @details If possible, removes the first agent of the street's queue, putting it in the destination node.
    /// If the agent is going into the destination node, it is removed from the simulation (and then reinserted if reinsert_agents is true)
    void m_evolveStreet(const std::unique_ptr<Street>& pStreet,
                        bool reinsert_agents) override;
    /// @brief If possible, removes one agent from the node, putting it on the next street.
    /// @param pNode A std::unique_ptr to the node
    /// @return bool True if the agent has been moved, false otherwise
    bool m_evolveNode(const std::unique_ptr<Node>& pNode) override;
    /// @brief Evolve the agents.
    /// @details Puts all new agents on a street, if possible, decrements all delays
    /// and increments all travel times.
    void m_evolveAgents() override;

  public:
    /// @brief Construct a new RoadDynamics object
    /// @param graph The graph representing the network
    /// @param useCache If true, the cache is used (default is false)
    /// @param seed The seed for the random number generator (default is std::nullopt)
    RoadDynamics(Graph& graph,
                 bool useCache = false,
                 std::optional<unsigned int> seed = std::nullopt);

    /// @brief Set the error probability
    /// @param errorProbability The error probability
    /// @throw std::invalid_argument If the error probability is not between 0 and 1
    void setErrorProbability(double errorProbability);

    void setPassageProbability(double passageProbability);
    /// @brief Set the force priorities flag
    /// @param forcePriorities The flag
    /// @details If true, if an agent cannot move to the next street, the whole node is skipped
    void setForcePriorities(bool forcePriorities) { m_forcePriorities = forcePriorities; }
    /// @brief Set the data update period.
    /// @param dataUpdatePeriod delay_t, The period
    /// @details Some data, i.e. the street queue lengths, are stored only after a fixed amount of time which is represented by this variable.
    void setDataUpdatePeriod(delay_t dataUpdatePeriod) {
      m_dataUpdatePeriod = dataUpdatePeriod;
    }

    /// @brief Add a set of agents to the simulation
    /// @param nAgents The number of agents to add
    /// @param uniformly If true, the agents are added uniformly on the streets
    /// @throw std::runtime_error If there are no itineraries
    void addAgentsUniformly(Size nAgents, std::optional<Id> itineraryId = std::nullopt);
    /// @brief Add a set of agents to the simulation
    /// @param nAgents The number of agents to add
    /// @param src_weights The weights of the source nodes
    /// @param dst_weights The weights of the destination nodes
    /// @throw std::invalid_argument If the source and destination nodes are the same
    template <typename TContainer>
      requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
               std::is_same_v<TContainer, std::map<Id, double>>)
    void addAgentsRandomly(Size nAgents,
                           const TContainer& src_weights,
                           const TContainer& dst_weights,
                           const size_t minNodeDistance = 0);

    /// @brief Evolve the simulation
    /// @details Evolve the simulation by moving the agents and updating the travel times.
    /// In particular:
    /// - Move the first agent of each street queue, if possible, putting it in the next node
    /// - Move the agents from each node, if possible, putting them in the next street and giving them a speed.
    /// If the error probability is not zero, the agents can move to a random street.
    /// If the agent is in the destination node, it is removed from the simulation (and then reinserted if reinsert_agents is true)
    /// - Cycle over agents and update their times
    /// @param reinsert_agents If true, the agents are reinserted in the simulation after they reach their destination
    void evolve(bool reinsert_agents = false) override;
    /// @brief Optimize the traffic lights by changing the green and red times
    /// @param threshold double, The minimum difference between green and red queues to trigger the optimization (n agents - default is 0)
    /// @param optimizationType TrafficLightOptimization, The type of optimization. Default is DOUBLE_TAIL
    /// @details The function cycles over the traffic lights and, if the difference between the two tails is greater than
    ///   the threshold multiplied by the mean capacity of the streets, it changes the green and red times of the traffic light, keeping the total cycle time constant.
    ///   The optimizationType parameter can be set to SINGLE_TAIL to use an algorith which looks only at the incoming street tails or to DOUBLE_TAIL to consider both incoming and outgoing street tails.
    void optimizeTrafficLights(double const threshold = 0.,
                               TrafficLightOptimization optimizationType =
                                   TrafficLightOptimization::DOUBLE_TAIL);
    /// @brief Get the mean travel time of the agents in \f$s\f$
    /// @param clearData If true, the travel times are cleared after the computation
    /// @return Measurement<double> The mean travel time of the agents and the standard deviation
    Measurement<double> meanTravelTime(bool clearData = false);
    /// @brief Get the mean travel distance of the agents in \f$m\f$
    /// @param clearData If true, the travel distances are cleared after the computation
    /// @return Measurement<double> The mean travel distance of the agents and the standard deviation
    Measurement<double> meanTravelDistance(bool clearData = false);
    /// @brief Get the mean travel speed of the agents in \f$m/s\f$
    /// @param clearData If true, the travel times and distances are cleared after the computation
    /// @return Measurement<double> The mean travel speed of the agents and the standard deviation
    Measurement<double> meanTravelSpeed(bool clearData = false);
    /// @brief Get the turn counts of the agents
    /// @return const std::array<unsigned long long, 3>& The turn counts
    /// @details The array contains the counts of left (0), straight (1), right (2) and U (3) turns
    const std::unordered_map<Id, std::array<unsigned long long, 4>>& turnCounts() const {
      return m_turnCounts;
    }
    /// @brief Get the turn probabilities of the agents
    /// @return std::array<double, 3> The turn probabilities
    /// @details The array contains the probabilities of left (0), straight (1), right (2) and U (3) turns
    std::unordered_map<Id, std::array<double, 4>> turnProbabilities(bool reset = true);

    std::unordered_map<Id, std::array<long, 4>> turnMapping() const {
      return m_turnMapping;
    }

    /// @brief Save the travel speeds of the agents in csv format
    /// @param filename The name of the file
    /// @param reset If true, the travel speeds are cleared after the computation
    void saveTravelSpeeds(const std::string& filename, bool reset = false);
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  RoadDynamics<delay_t>::RoadDynamics(Graph& graph,
                                      bool useCache,
                                      std::optional<unsigned int> seed)
      : Dynamics<Agent<delay_t>>(graph, useCache, seed),
        m_previousOptimizationTime{0},
        m_errorProbability{std::nullopt},
        m_passageProbability{std::nullopt},
        m_forcePriorities{false} {
    for (const auto& [streetId, street] : this->m_graph.streetSet()) {
      m_streetTails.emplace(streetId, 0);
      m_turnCounts.emplace(streetId, std::array<unsigned long long, 4>{0, 0, 0, 0});
      // fill turn mapping as [streetId, [left street Id, straight street Id, right street Id, U self street Id]]
      m_turnMapping.emplace(streetId, std::array<long, 4>{-1, -1, -1, -1});
      // Turn mappings
      const auto& srcNodeId = street->target();
      for (const auto& targetId : this->m_graph.adjMatrix().getRow(srcNodeId)) {
        auto const ss = srcNodeId * this->m_graph.nNodes() + targetId;
        const auto& delta = street->angle() - this->m_graph.streetSet()[ss]->angle();
        if (std::abs(delta) < std::numbers::pi) {
          if (delta < 0.) {
            m_turnMapping[streetId][dsm::Direction::RIGHT] = ss;
            ;  // right
          } else if (delta > 0.) {
            m_turnMapping[streetId][dsm::Direction::LEFT] = ss;  // left
          } else {
            m_turnMapping[streetId][dsm::Direction::STRAIGHT] = ss;  // straight
          }
        } else {
          m_turnMapping[streetId][dsm::Direction::UTURN] = ss;  // U
        }
      }
      m_adjMatrix_T = this->m_graph.adjMatrix();
      m_adjMatrix_T.transpose();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Id RoadDynamics<delay_t>::m_nextStreetId(Id agentId,
                                           Id nodeId,
                                           std::optional<Id> streetId) {
    auto const& pAgent{this->agents().at(agentId)};
    auto possibleMoves = this->m_graph.adjMatrix().getRow(nodeId);
    if (!pAgent->isRandom()) {
      std::uniform_real_distribution<double> uniformDist{0., 1.};
      if (!(this->itineraries().empty())) {
        if (!(m_errorProbability.has_value() &&
              uniformDist(this->m_generator) < m_errorProbability)) {
          const auto& it = this->itineraries().at(pAgent->itineraryId());
          if (it->destination() != nodeId) {
            possibleMoves = it->path()->getRow(nodeId);
          }
        }
      }
    }
    assert(possibleMoves.size() > 0);
    std::uniform_int_distribution<Size> moveDist{
        0, static_cast<Size>(possibleMoves.size() - 1)};
    // while loop to avoid U turns in non-roundabout junctions
    Id nextStreetId;
    do {
      nextStreetId =
          nodeId * this->m_graph.nNodes() + possibleMoves[moveDist(this->m_generator)];
    } while (!this->m_graph.node(nodeId)->isRoundabout() && streetId.has_value() &&
             (this->m_graph.street(nextStreetId)->target() ==
              this->m_graph.street(streetId.value())->source()) &&
             (possibleMoves.size() > 1));
    return nextStreetId;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_increaseTurnCounts(Id streetId, double delta) {
    if (std::abs(delta) < std::numbers::pi) {
      if (delta < 0.) {
        ++m_turnCounts[streetId][0];  // right
      } else if (delta > 0.) {
        ++m_turnCounts[streetId][2];  // left
      } else {
        ++m_turnCounts[streetId][1];  // straight
      }
    } else {
      ++m_turnCounts[streetId][3];  // U
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveStreet(const std::unique_ptr<Street>& pStreet,
                                             bool reinsert_agents) {
    auto const nLanes = pStreet->nLanes();
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    bool bCanPass{true};
    if (pStreet->isStochastic() &&
        (uniformDist(this->m_generator) >
         dynamic_cast<StochasticStreet&>(*pStreet).flowRate())) {
      bCanPass = false;
    }
    for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
      if (pStreet->queue(queueIndex).empty()) {
        continue;
      }
      const auto agentId{pStreet->queue(queueIndex).front()};
      auto const& pAgent{this->agents().at(agentId)};
      if (pAgent->delay() > 0) {
        continue;
      }
      pAgent->setSpeed(0.);
      const auto& destinationNode{this->m_graph.node(pStreet->target())};
      if (destinationNode->isFull()) {
        continue;
      }
      if (destinationNode->isTrafficLight()) {
        auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
        auto const direction{pStreet->laneMapping().at(queueIndex)};
        if (!tl.isGreen(pStreet->id(), direction)) {
          continue;
        }
      }
      bCanPass = bCanPass &&
                 (uniformDist(this->m_generator) < m_passageProbability.value_or(1.1));
      bool bArrived{false};
      if (!bCanPass) {
        if (pAgent->isRandom()) {
          bArrived = true;
        } else {
          continue;
        }
      }
      if (!pAgent->isRandom()) {
        if (destinationNode->id() ==
            this->itineraries().at(pAgent->itineraryId())->destination()) {
          bArrived = true;
        }
      }
      if (bArrived) {
        if (pStreet->dequeue(queueIndex) == std::nullopt) {
          continue;
        }
        m_travelDTs.push_back({pAgent->distance(), static_cast<double>(pAgent->time())});
        if (reinsert_agents) {
          // reset Agent's values
          pAgent->reset();
        } else {
          this->removeAgent(agentId);
        }
        continue;
      }
      auto const& nextStreet{
          this->m_graph.street(this->agents().at(agentId)->nextStreetId().value())};
      if (nextStreet->isFull()) {
        continue;
      }
      if (pStreet->dequeue(queueIndex) == std::nullopt) {
        continue;
      }
      assert(destinationNode->id() == nextStreet->source());
      if (destinationNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
        auto const delta{nextStreet->deltaAngle(pStreet->angle())};
        m_increaseTurnCounts(pStreet->id(), delta);
        intersection.addAgent(delta, agentId);
      } else if (destinationNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*destinationNode);
        roundabout.enqueue(agentId);
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  bool RoadDynamics<delay_t>::m_evolveNode(const std::unique_ptr<Node>& pNode) {
    if (pNode->isIntersection()) {
      auto& intersection = dynamic_cast<Intersection&>(*pNode);
      if (intersection.agents().empty()) {
        return false;
      }
      for (auto const [angle, agentId] : intersection.agents()) {
        auto const& nextStreet{
            this->m_graph.street(this->agents().at(agentId)->nextStreetId().value())};
        if (nextStreet->isFull()) {
          if (m_forcePriorities) {
            return false;
          }
          continue;
        }
        intersection.removeAgent(agentId);
        this->agents().at(agentId)->setStreetId(nextStreet->id());
        this->setAgentSpeed(agentId);
        this->agents().at(agentId)->incrementDelay(
            std::ceil(nextStreet->length() / this->agents().at(agentId)->speed()));
        nextStreet->addAgent(agentId);
        return true;
      }
      return false;
    } else if (pNode->isRoundabout()) {
      auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
      if (roundabout.agents().empty()) {
        return false;
      }
      auto const agentId{roundabout.agents().front()};
      auto const& nextStreet{
          this->m_graph.street(this->agents().at(agentId)->nextStreetId().value())};
      if (!(nextStreet->isFull())) {
        if (this->agents().at(agentId)->streetId().has_value()) {
          const auto streetId = this->agents().at(agentId)->streetId().value();
          auto delta = nextStreet->angle() - this->m_graph.streetSet()[streetId]->angle();
          if (delta > std::numbers::pi) {
            delta -= 2 * std::numbers::pi;
          } else if (delta < -std::numbers::pi) {
            delta += 2 * std::numbers::pi;
          }
          m_increaseTurnCounts(streetId, delta);
        }
        roundabout.dequeue();
        this->agents().at(agentId)->setStreetId(nextStreet->id());
        this->setAgentSpeed(agentId);
        this->agents().at(agentId)->incrementDelay(
            std::ceil(nextStreet->length() / this->agents().at(agentId)->speed()));
        nextStreet->addAgent(agentId);
      } else {
        return false;
      }
    }
    return true;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveAgents() {
    std::uniform_int_distribution<Id> nodeDist{
        0, static_cast<Id>(this->m_graph.nNodes() - 1)};
    for (const auto& [agentId, agent] : this->agents()) {
      if (agent->delay() > 0) {
        const auto& street{this->m_graph.streetSet()[agent->streetId().value()]};
        if (agent->delay() > 1) {
          agent->incrementDistance();
        } else {
          double distance{std::fmod(street->length(), agent->speed())};
          if (distance < std::numeric_limits<double>::epsilon()) {
            agent->incrementDistance();
          } else {
            agent->incrementDistance(distance);
          }
        }
        agent->decrementDelay();
        if (agent->delay() == 0) {
          auto const nLanes = street->nLanes();
          bool bArrived{false};
          if (!agent->isRandom()) {
            if (this->itineraries().at(agent->itineraryId())->destination() ==
                street->nodePair().second) {
              agent->updateItinerary();
            }
            if (this->itineraries().at(agent->itineraryId())->destination() ==
                street->nodePair().second) {
              bArrived = true;
            }
          }
          if (bArrived) {
            std::uniform_int_distribution<size_t> laneDist{
                0, static_cast<size_t>(nLanes - 1)};
            street->enqueue(agentId, laneDist(this->m_generator));
          } else {
            auto const nextStreetId =
                this->m_nextStreetId(agentId, street->target(), street->id());
            auto const& pNextStreet{this->m_graph.street(nextStreetId)};
            agent->setNextStreetId(nextStreetId);
            if (nLanes == 1) {
              street->enqueue(agentId, 0);
            } else {
              auto const deltaAngle{pNextStreet->deltaAngle(street->angle())};
              if (std::abs(deltaAngle) < std::numbers::pi) {
                // Lanes are counted as 0 is the far right lane
                if (std::abs(deltaAngle) < std::numbers::pi / 4) {
                  std::vector<double> weights;
                  for (auto const& queue : street->exitQueues()) {
                    weights.push_back(1. / (queue.size() + 1));
                  }
                  // If all weights are the same, make the last 0
                  if (std::all_of(weights.begin(), weights.end(), [&](double w) {
                        return std::abs(w - weights.front()) <
                               std::numeric_limits<double>::epsilon();
                      })) {
                    weights.back() = 0.;
                    if (nLanes > 2) {
                      weights.front() = 0.;
                    }
                  }
                  // Normalize the weights
                  auto const sum = std::accumulate(weights.begin(), weights.end(), 0.);
                  for (auto& w : weights) {
                    w /= sum;
                  }
                  std::discrete_distribution<size_t> laneDist{weights.begin(),
                                                              weights.end()};
                  street->enqueue(agentId, laneDist(this->m_generator));
                } else if (deltaAngle < 0.) {            // Right
                  street->enqueue(agentId, 0);           // Always the first lane
                } else {                                 // Left (deltaAngle > 0.)
                  street->enqueue(agentId, nLanes - 1);  // Always the last lane
                }
              } else {                                 // U turn
                street->enqueue(agentId, nLanes - 1);  // Always the last lane
              }
            }
          }
        }
      } else if (!agent->streetId().has_value() && !agent->nextStreetId().has_value()) {
        Id srcNodeId = agent->srcNodeId().has_value() ? agent->srcNodeId().value()
                                                      : nodeDist(this->m_generator);
        const auto& srcNode{this->m_graph.nodeSet()[srcNodeId]};
        if (srcNode->isFull()) {
          continue;
        }
        const auto& nextStreet{
            this->m_graph.streetSet()[this->m_nextStreetId(agentId, srcNode->id())]};
        if (nextStreet->isFull()) {
          continue;
        }
        assert(srcNode->id() == nextStreet->nodePair().first);
        if (srcNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*srcNode);
          intersection.addAgent(0., agentId);
        } else if (srcNode->isRoundabout()) {
          auto& roundabout = dynamic_cast<Roundabout&>(*srcNode);
          roundabout.enqueue(agentId);
        }
        agent->setNextStreetId(nextStreet->id());
      } else if (agent->delay() == 0) {
        agent->setSpeed(0.);
      }
      agent->incrementTime();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setErrorProbability(double errorProbability) {
    if (errorProbability < 0. || errorProbability > 1.) {
      Logger::error(
          std::format("The error probability ({}) must be in [0, 1]", errorProbability));
    }
    m_errorProbability = errorProbability;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setPassageProbability(double passageProbability) {
    if (passageProbability < 0. || passageProbability > 1.) {
      Logger::error(std::format("The passage probability ({}) must be between 0 and 1",
                                passageProbability));
    }
    m_passageProbability = passageProbability;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgentsUniformly(Size nAgents,
                                                 std::optional<Id> optItineraryId) {
    if (this->itineraries().empty()) {
      // TODO: make this possible for random agents
      throw std::invalid_argument(Logger::buildExceptionMessage(
          "It is not possible to add random agents without itineraries."));
    }
    Id itineraryId{0};
    const bool randomItinerary{!optItineraryId.has_value()};
    if (!randomItinerary) {
      itineraryId = optItineraryId.value();
    }
    std::uniform_int_distribution<Size> itineraryDist{
        0, static_cast<Size>(this->itineraries().size() - 1)};
    std::uniform_int_distribution<Size> streetDist{
        0, static_cast<Size>(this->m_graph.nEdges() - 1)};
    for (Size i{0}; i < nAgents; ++i) {
      if (randomItinerary) {
        auto itineraryIt{this->itineraries().cbegin()};
        std::advance(itineraryIt, itineraryDist(this->m_generator));
        itineraryId = itineraryIt->first;
      }
      Id agentId{0};
      if (!(this->agents().empty())) {
        agentId = this->agents().rbegin()->first + 1;
      }
      Id streetId{0};
      do {
        auto streetIt = this->m_graph.streetSet().begin();
        Size step = streetDist(this->m_generator);
        std::advance(streetIt, step);
        streetId = streetIt->first;
      } while (this->m_graph.streetSet()[streetId]->isFull() &&
               this->nAgents() < this->m_graph.maxCapacity());
      const auto& street{this->m_graph.streetSet()[streetId]};
      this->addAgent(agentId, itineraryId, street->nodePair().first);
      auto const& pAgent{this->agents().at(agentId)};
      pAgent->setStreetId(streetId);
      this->setAgentSpeed(agentId);
      pAgent->incrementDelay(
          std::ceil(street->length() / this->agents().at(agentId)->speed()));
      street->addAgent(agentId);
      ++agentId;
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
             std::is_same_v<TContainer, std::map<Id, double>>)
  void RoadDynamics<delay_t>::addAgentsRandomly(Size nAgents,
                                                const TContainer& src_weights,
                                                const TContainer& dst_weights,
                                                const size_t minNodeDistance) {
    if (src_weights.size() == 1 && dst_weights.size() == 1 &&
        src_weights.begin()->first == dst_weights.begin()->first) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("The only source node {} is also the only destination node.",
                      src_weights.begin()->first)));
    }
    auto const srcSum{std::accumulate(
        src_weights.begin(),
        src_weights.end(),
        0.,
        [](double sum, const std::pair<Id, double>& p) {
          if (p.second < 0.) {
            Logger::error(std::format(
                "Negative weight ({}) for source node {}.", p.second, p.first));
          }
          return sum + p.second;
        })};
    auto const dstSum{std::accumulate(
        dst_weights.begin(),
        dst_weights.end(),
        0.,
        [](double sum, const std::pair<Id, double>& p) {
          if (p.second < 0.) {
            Logger::error(std::format(
                "Negative weight ({}) for destination node {}.", p.second, p.first));
          }
          return sum + p.second;
        })};
    std::uniform_real_distribution<double> srcUniformDist{0., srcSum};
    std::uniform_real_distribution<double> dstUniformDist{0., dstSum};
    Id agentId{0};
    if (!this->agents().empty()) {
      agentId = this->agents().rbegin()->first + 1;
    }
    while (nAgents > 0) {
      Id srcId{0}, dstId{0};
      if (dst_weights.size() == 1) {
        dstId = dst_weights.begin()->first;
        srcId = dstId;
      }
      double dRand, sum;
      while (srcId == dstId) {
        dRand = srcUniformDist(this->m_generator);
        sum = 0.;
        for (const auto& [id, weight] : src_weights) {
          srcId = id;
          sum += weight;
          if (dRand < sum) {
            break;
          }
        }
      }
      if (src_weights.size() > 1) {
        dstId = srcId;
      }
      while (dstId == srcId) {
        dRand = dstUniformDist(this->m_generator);
        sum = 0.;
        for (const auto& [id, weight] : dst_weights) {
          // if the node is at a minimum distance from the destination, skip it
          auto result{this->m_graph.shortestPath(srcId, id)};
          if (result.has_value() && result.value().path().size() < minNodeDistance &&
              dst_weights.size() > 1) {
            continue;
          }
          dstId = id;
          sum += weight;
          if (dRand < sum) {
            break;
          }
        }
      }
      // find the itinerary with the given destination as destination
      auto itineraryIt{std::find_if(this->itineraries().cbegin(),
                                    this->itineraries().cend(),
                                    [dstId](const auto& itinerary) {
                                      return itinerary.second->destination() == dstId;
                                    })};
      if (itineraryIt == this->itineraries().cend()) {
        Logger::error(std::format("Itinerary with destination {} not found.", dstId));
      }
      this->addAgent(agentId, itineraryIt->first, srcId);
      --nAgents;
      ++agentId;
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::evolve(bool reinsert_agents) {
    // move the first agent of each street queue, if possible, putting it in the next node
    bool const bUpdateData =
        m_dataUpdatePeriod.has_value() && this->m_time % m_dataUpdatePeriod.value() == 0;
    auto const N{this->m_graph.nNodes()};
    for (auto const& [nodeId, _] : this->m_graph.nodeSet()) {
      for (auto const& srcNodeId : m_adjMatrix_T.getRow(nodeId)) {
        auto const streetId{srcNodeId * N + nodeId};
        auto const& pStreet{this->m_graph.street(streetId)};
        // Logger::info(std::format("Evolving street {}", streetId));
        if (bUpdateData) {
          m_streetTails[streetId] += pStreet->nExitingAgents();
        }
        for (auto i = 0; i < pStreet->transportCapacity(); ++i) {
          this->m_evolveStreet(pStreet, reinsert_agents);
        }
      }
    }
    // Move transport capacity agents from each node
    for (const auto& [nodeId, pNode] : this->m_graph.nodeSet()) {
      for (auto i = 0; i < pNode->transportCapacity(); ++i) {
        if (!this->m_evolveNode(pNode)) {
          break;
        }
      }
      if (pNode->isTrafficLight()) {
        auto& tl = dynamic_cast<TrafficLight&>(*pNode);
        ++tl;  // Increment the counter
      }
    }
    // cycle over agents and update their times
    this->m_evolveAgents();
    // increment time simulation
    ++this->m_time;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::optimizeTrafficLights(
      double const threshold, TrafficLightOptimization const optimizationType) {
    if (threshold < 0) {
      Logger::error(
          std::format("The threshold parameter ({}) must be greater than 0.", threshold));
    }
    auto const nCycles{static_cast<double>(this->m_time - m_previousOptimizationTime) /
                       m_dataUpdatePeriod.value()};
    for (const auto& [nodeId, pNode] : this->m_graph.nodeSet()) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);
      auto const& streetPriorities = tl.streetPriorities();
      auto const meanGreenFraction{tl.meanGreenTime(true) / tl.cycleTime()};
      auto const meanRedFraction{tl.meanGreenTime(false) / tl.cycleTime()};

      double inputGreenSum{0.}, inputRedSum{0.};
      auto const N{this->m_graph.nNodes()};
      auto column = m_adjMatrix_T.getRow(nodeId);
      for (const auto& sourceId : column) {
        auto const streetId = sourceId * N + nodeId;
        auto const& pStreet{this->m_graph.street(streetId)};
        if (streetPriorities.contains(streetId)) {
          inputGreenSum += m_streetTails[streetId] / pStreet->nLanes();
        } else {
          inputRedSum += m_streetTails[streetId] / pStreet->nLanes();
        }
      }
      inputGreenSum /= meanGreenFraction;
      inputRedSum /= meanRedFraction;
      // std::clog << std::format("Traffic Light: {} - Green: {} - Red: {}\n",
      //                          nodeId,
      //                          inputGreenSum,
      //                          inputRedSum);
      auto const inputDifference{(inputGreenSum - inputRedSum) / nCycles};
      delay_t const delta = std::round(std::abs(inputDifference) / column.size());
      // std::clog << std::format("TL: {}, current delta {}, difference: {}",
      //                          nodeId,
      //                          delta,
      //                          inputDifference)
      //           << std::endl;
      auto const greenTime = tl.minGreenTime(true);
      auto const redTime = tl.minGreenTime(false);
      if (optimizationType == TrafficLightOptimization::SINGLE_TAIL) {
        if (delta == 0 || std::abs(inputDifference) < threshold) {
          tl.resetCycles();
          continue;
        }
        // std::clog << std::format("TL: {}, difference: {}, red time: {}",
        //                          nodeId,
        //                          inputDifference,
        //                          redTime)
        //           << std::endl;
        if ((inputDifference > 0) && (redTime > delta)) {
          tl.increaseGreenTimes(delta);
        } else if ((inputDifference < 0) && (greenTime > delta)) {
          tl.decreaseGreenTimes(delta);
        }
      } else if (optimizationType == TrafficLightOptimization::DOUBLE_TAIL) {
        // If the difference is not less than the threshold
        //    - Check that the incoming streets have a density less than the mean one (eventually + tolerance): I want to avoid being into the cluster, better to be out or on the border
        //    - If the previous check fails, do nothing
        double outputGreenSum{0.}, outputRedSum{0.};
        for (const auto& targetId : this->m_graph.adjMatrix().getRow(nodeId)) {
          auto const streetId = nodeId * N + targetId;
          auto const& pStreet{this->m_graph.street(streetId)};
          if (streetPriorities.contains(streetId)) {
            outputGreenSum += m_streetTails[streetId] / pStreet->nLanes();
          } else {
            outputRedSum += m_streetTails[streetId] / pStreet->nLanes();
          }
        }
        auto const outputDifference{(outputGreenSum - outputRedSum) / nCycles};
        if ((inputDifference * outputDifference > 0) ||
            std::max(std::abs(inputDifference), std::abs(outputDifference)) < threshold ||
            delta == 0) {
          tl.resetCycles();
          continue;
        }
        if (std::abs(inputDifference) > std::abs(outputDifference)) {
          if ((inputDifference > 0) && (redTime > delta)) {
            tl.increaseGreenTimes(delta);
          } else if ((inputDifference < 0) && (greenTime > delta)) {
            tl.decreaseGreenTimes(delta);
          }
        } else {
          if ((outputDifference < 0) && (redTime > delta)) {
            tl.increaseGreenTimes(delta);
          } else if ((outputDifference > 0) && (greenTime > delta)) {
            tl.decreaseGreenTimes(delta);
          }
        }
      }
    }
    // Cleaning variables
    for (auto& [id, element] : m_streetTails) {
      element = 0.;
    }
    m_previousOptimizationTime = this->m_time;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::meanTravelTime(bool clearData) {
    std::vector<double> travelTimes;
    if (!m_travelDTs.empty()) {
      travelTimes.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        travelTimes.push_back(time);
      }
      if (clearData) {
        m_travelDTs.clear();
      }
    }
    return Measurement<double>(travelTimes);
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::meanTravelDistance(bool clearData) {
    std::vector<double> travelDistances;
    if (!m_travelDTs.empty()) {
      travelDistances.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        travelDistances.push_back(distance);
      }
      if (clearData) {
        m_travelDTs.clear();
      }
    }
    return Measurement<double>(travelDistances);
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::meanTravelSpeed(bool clearData) {
    std::vector<double> travelSpeeds;
    travelSpeeds.reserve(m_travelDTs.size());
    for (auto const& [distance, time] : m_travelDTs) {
      travelSpeeds.push_back(distance / time);
    }
    if (clearData) {
      m_travelDTs.clear();
    }
    return Measurement<double>(travelSpeeds);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  std::unordered_map<Id, std::array<double, 4>> RoadDynamics<delay_t>::turnProbabilities(
      bool reset) {
    std::unordered_map<Id, std::array<double, 4>> res;
    for (auto& [streetId, counts] : m_turnCounts) {
      std::array<double, 4> probabilities{0., 0., 0., 0.};
      const auto sum{std::accumulate(counts.cbegin(), counts.cend(), 0.)};
      if (sum != 0) {
        for (auto i{0}; i < counts.size(); ++i) {
          probabilities[i] = counts[i] / sum;
        }
      }
      res.emplace(streetId, probabilities);
    }
    if (reset) {
      for (auto& [streetId, counts] : m_turnCounts) {
        std::fill(counts.begin(), counts.end(), 0);
      }
    }
    return res;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveTravelSpeeds(const std::string& filename, bool reset) {
    bool bEmptyFile{false};
    {
      std::ifstream file(filename);
      bEmptyFile = file.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      Logger::error(std::format("Error opening file \"{}\" for writing.", filename));
    }
    if (bEmptyFile) {
      file << "time;speeds" << std::endl;
    }
    file << this->time() << ';';
    for (auto it = m_travelDTs.cbegin(); it != m_travelDTs.cend(); ++it) {
      file << std::fixed << std::setprecision(2) << it->first / it->second;
      if (it != m_travelDTs.cend() - 1) {
        file << ',';
      }
    }
    file << std::endl;
    file.close();
    if (reset) {
      m_travelDTs.clear();
    }
  }
};  // namespace dsm
