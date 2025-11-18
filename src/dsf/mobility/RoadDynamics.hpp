/// @file       /src/dsf/headers/RoadDynamics.hpp
/// @brief      Defines the RoadDynamics class.
///
/// @details    This file contains the definition of the RoadDynamics class.
///             The RoadDynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <exception>
#include <format>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <optional>
#include <random>
#include <span>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include <tbb/tbb.h>
#include <spdlog/spdlog.h>

#include "../base/Dynamics.hpp"
#include "Agent.hpp"
#include "Itinerary.hpp"
#include "RoadNetwork.hpp"
#include "../utility/Typedef.hpp"

static auto constexpr g_cacheFolder = "./.dsfcache/";

namespace dsf::mobility {
  /// @brief The RoadDynamics class represents the dynamics of the network.
  /// @tparam delay_t The type of the agent's delay
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  class RoadDynamics : public Dynamics<RoadNetwork> {
    std::vector<Id> m_nodeIndices;
    std::vector<std::unique_ptr<Agent>> m_agents;
    std::unordered_map<Id, std::unique_ptr<Itinerary>> m_itineraries;
    std::unordered_map<Id, double> m_originNodes;
    std::unordered_map<Id, double> m_destinationNodes;
    Size m_nAgents;

  protected:
    std::unordered_map<Id, std::unordered_map<Id, size_t>> m_turnCounts;
    std::unordered_map<Id, std::array<long, 4>> m_turnMapping;
    tbb::concurrent_unordered_map<Id, std::unordered_map<Direction, double>>
        m_queuesAtTrafficLights;
    tbb::concurrent_vector<std::pair<double, double>> m_travelDTs;
    std::time_t m_previousOptimizationTime;

  private:
    std::function<double(std::unique_ptr<Street> const&)> m_weightFunction;
    std::optional<double> m_errorProbability;
    std::optional<double> m_passageProbability;
    double m_maxTravelDistance;
    std::time_t m_maxTravelTime;
    double m_weightTreshold;
    std::optional<double> m_timeToleranceFactor;
    std::optional<delay_t> m_dataUpdatePeriod;
    bool m_bCacheEnabled;
    bool m_forcePriorities;

  private:
    /// @brief Update the path of a single itinerary using Dijsktra's algorithm
    /// @param pItinerary An std::unique_prt to the itinerary
    void m_updatePath(std::unique_ptr<Itinerary> const& pItinerary);

    /// @brief Get the next street id
    /// @param agentId The id of the agent
    /// @param NodeId The id of the node
    /// @param streetId The id of the incoming street
    /// @return Id The id of the randomly selected next street
    virtual Id m_nextStreetId(std::unique_ptr<Agent> const& pAgent,
                              Id NodeId,
                              std::optional<Id> streetId = std::nullopt);
    /// @brief Evolve a street
    /// @param pStreet A std::unique_ptr to the street
    /// @param reinsert_agents If true, the agents are reinserted in the simulation after they reach their destination
    /// @details If possible, removes the first agent of the street's queue, putting it in the destination node.
    /// If the agent is going into the destination node, it is removed from the simulation (and then reinserted if reinsert_agents is true)
    void m_evolveStreet(std::unique_ptr<Street> const& pStreet, bool reinsert_agents);
    /// @brief If possible, removes one agent from the node, putting it on the next street.
    /// @param pNode A std::unique_ptr to the node
    void m_evolveNode(const std::unique_ptr<RoadJunction>& pNode);
    /// @brief Evolve the agents.
    /// @details Puts all new agents on a street, if possible, decrements all delays
    /// and increments all travel times.
    void m_evolveAgents();

    void m_trafficlightSingleTailOptimizer(double const& beta,
                                           std::optional<std::ofstream>& logStream);

    virtual double m_streetEstimatedTravelTime(
        std::unique_ptr<Street> const& pStreet) const = 0;

  public:
    /// @brief Construct a new RoadDynamics object
    /// @param graph The graph representing the network
    /// @param useCache If true, the cache is used (default is false)
    /// @param seed The seed for the random number generator (default is std::nullopt)
    /// @param weightFunction The dsf::PathWeight function to use for the pathfinding (default is dsf::PathWeight::TRAVELTIME)
    /// @param weightTreshold The weight treshold for updating the paths (default is std::nullopt)
    RoadDynamics(RoadNetwork& graph,
                 bool useCache = false,
                 std::optional<unsigned int> seed = std::nullopt,
                 PathWeight const weightFunction = PathWeight::TRAVELTIME,
                 std::optional<double> weightTreshold =
                     std::nullopt);  // 60 seconds thresholds for paths

    /// @brief Set the error probability
    /// @param errorProbability The error probability
    /// @throw std::invalid_argument If the error probability is not between 0 and 1
    void setErrorProbability(double errorProbability);
    /// @brief Set the passage probability
    /// @param passageProbability The passage probability
    /// @details The passage probability is the probability of passing through a node
    ///   It is useful in the case of random agents
    void setPassageProbability(double passageProbability);
    /// @brief Set the time tolerance factor for killing stagnant agents.
    ///   An agent will be considered stagnant if it has not moved for timeToleranceFactor * std::ceil(street_length / street_maxSpeed) time units.
    /// @param timeToleranceFactor The time tolerance factor
    /// @throw std::invalid_argument If the time tolerance factor is not positive
    void killStagnantAgents(double timeToleranceFactor = 3.);
    /// @brief Set the weight function
    /// @param pathWeight The dsf::PathWeight function to use for the pathfinding
    /// @param weightThreshold The weight threshold for updating the paths (default is std::nullopt)
    void setWeightFunction(PathWeight const pathWeight,
                           std::optional<double> weightThreshold = std::nullopt);
    /// @brief Set the force priorities flag
    /// @param forcePriorities The flag
    /// @details If true, if an agent cannot move to the next street, the whole node is skipped
    inline void setForcePriorities(bool forcePriorities) noexcept {
      m_forcePriorities = forcePriorities;
    }
    /// @brief Set the data update period.
    /// @param dataUpdatePeriod delay_t, The period
    /// @details Some data, i.e. the street queue lengths, are stored only after a fixed amount of time which is represented by this variable.
    inline void setDataUpdatePeriod(delay_t dataUpdatePeriod) noexcept {
      m_dataUpdatePeriod = dataUpdatePeriod;
    }
    /// @brief Set the maximum distance which a random agent can travel
    /// @param maxDistance The maximum distance
    /// @throw std::invalid_argument If the maximum distance is negative
    inline void setMaxDistance(double const maxDistance) {
      if (maxDistance < 0.) {
        throw std::invalid_argument(std::format(
            "Maximum travel distance ({}) must be non-negative", maxDistance));
      }
      m_maxTravelDistance = maxDistance;
    };
    /// @brief Set the maximum travel time which a random agent can travel
    /// @param maxTravelTime The maximum travel time
    inline void setMaxTravelTime(std::time_t const maxTravelTime) noexcept {
      m_maxTravelTime = maxTravelTime;
    };
    void setOriginNodes(std::unordered_map<Id, double> const& originNodes);

    void setDestinationNodes(std::unordered_map<Id, double> const& destinationNodes);
    /// @brief Set the destination nodes
    /// @param destinationNodes The destination nodes (as an initializer list)
    void setDestinationNodes(std::initializer_list<Id> destinationNodes);
    /// @brief Set the destination nodes
    /// @param destinationNodes A container of destination nodes ids
    /// @details The container must have a value_type convertible to Id and begin() and end() methods
    template <typename TContainer>
      requires(std::is_convertible_v<typename TContainer::value_type, Id>)
    void setDestinationNodes(TContainer const& destinationNodes);

    virtual void setAgentSpeed(std::unique_ptr<Agent> const& pAgent) = 0;
    /// @brief Initialize the turn counts map
    /// @throws std::runtime_error if the turn counts map is already initialized
    void initTurnCounts();
    /// @brief Reset the turn counts map values to zero
    /// @throws std::runtime_error if the turn counts map is not initialized
    void resetTurnCounts();

    /// @brief Update the paths of the itineraries based on the given weight function
    void updatePaths();
    /// @brief Add agents uniformly on the road network
    /// @param nAgents The number of agents to add
    /// @param itineraryId The id of the itinerary to use (default is std::nullopt)
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
                           const TContainer& dst_weights);

    void addAgentsRandomly(Size nAgents);

    /// @brief Add an agent to the simulation
    /// @param agent std::unique_ptr to the agent
    void addAgent(std::unique_ptr<Agent> agent);

    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
    void addAgent(TArgs&&... args);

    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
    void addAgents(Size nAgents, TArgs&&... args);

    /// @brief Add an itinerary
    /// @param ...args The arguments to construct the itinerary
    /// @details The arguments must be compatible with any constructor of the Itinerary class
    template <typename... TArgs>
      requires(std::is_constructible_v<Itinerary, TArgs...>)
    void addItinerary(TArgs&&... args);
    /// @brief Add an itinerary
    /// @param itinerary std::unique_ptr to the itinerary
    /// @throws std::invalid_argument If the itinerary already exists
    /// @throws std::invalid_argument If the itinerary's destination is not a node of the graph
    void addItinerary(std::unique_ptr<Itinerary> itinerary);

    /// @brief Evolve the simulation
    /// @details Evolve the simulation by moving the agents and updating the travel times.
    /// In particular:
    /// - Move the first agent of each street queue, if possible, putting it in the next node
    /// - Move the agents from each node, if possible, putting them in the next street and giving them a speed.
    /// If the error probability is not zero, the agents can move to a random street.
    /// If the agent is in the destination node, it is removed from the simulation (and then reinserted if reinsert_agents is true)
    /// - Cycle over agents and update their times
    /// @param reinsert_agents If true, the agents are reinserted in the simulation after they reach their destination
    void evolve(bool reinsert_agents = false);
    /// @brief Optimize the traffic lights by changing the green and red times
    /// @param optimizationType TrafficLightOptimization, The type of optimization. Default is DOUBLE_TAIL
    /// @param logFile The file into which write the logs (default is empty, meaning no logging)
    /// @param percentage double, the maximum amount (percentage) of the green time to change (default is 0.3)
    /// @param threshold double, The ratio between the self-density and neighbour density to trigger the non-local optimization (default is 1.3)
    /// @details The local optimization is done by changing the green time of each traffic light, trying to make it proportional to the
    ///    queue lengths at each phase. The non-local optimization is done by synchronizing the traffic lights which are congested over threshold.
    void optimizeTrafficLights(
        TrafficLightOptimization optimizationType = TrafficLightOptimization::DOUBLE_TAIL,
        const std::string& logFile = std::string(),
        double const percentage = 0.3,
        double const threshold = 1.3);

    /// @brief Get the itineraries
    /// @return const std::unordered_map<Id, Itinerary>&, The itineraries
    inline const std::unordered_map<Id, std::unique_ptr<Itinerary>>& itineraries()
        const noexcept {
      return m_itineraries;
    }
    /// @brief Get the origin nodes of the graph
    /// @return std::unordered_map<Id, double> const& The origin nodes of the graph
    inline std::unordered_map<Id, double> const& originNodes() const noexcept {
      return m_originNodes;
    }
    /// @brief Get the origin nodes of the graph
    /// @return std::unordered_map<Id, double>& The origin nodes of the graph
    inline std::unordered_map<Id, double>& originNodes() noexcept {
      return m_originNodes;
    }
    /// @brief Get the destination nodes of the graph
    /// @return std::unordered_map<Id, double> const& The destination nodes of the graph
    inline std::unordered_map<Id, double> const& destinationNodes() const noexcept {
      return m_destinationNodes;
    }
    /// @brief Get the destination nodes of the graph
    /// @return std::unordered_map<Id, double>& The destination nodes of the graph
    inline std::unordered_map<Id, double>& destinationNodes() noexcept {
      return m_destinationNodes;
    }
    /// @brief Get the agents
    /// @return const std::unordered_map<Id, Agent<Id>>&, The agents
    inline const std::vector<std::unique_ptr<Agent>>& agents() const noexcept {
      return m_agents;
    }
    /// @brief Get the number of agents currently in the simulation
    /// @return Size The number of agents
    Size nAgents() const;

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
    /// @return const std::unordered_map<Id, std::unordered_map<Id, size_t>>& The turn counts. The outer map's key is the street id, the inner map's key is the next street id and the value is the number of counts
    inline std::unordered_map<Id, std::unordered_map<Id, size_t>> const& turnCounts()
        const noexcept {
      return m_turnCounts;
    };
    /// @brief Get the normalized turn counts of the agents
    /// @return const std::unordered_map<Id, std::unordered_map<Id, double>>& The normalized turn counts. The outer map's key is the street id, the inner map's key is the next street id and the value is the normalized number of counts
    std::unordered_map<Id, std::unordered_map<Id, double>> const normalizedTurnCounts()
        const noexcept;

    std::unordered_map<Id, std::array<long, 4>> turnMapping() const {
      return m_turnMapping;
    }

    virtual double streetMeanSpeed(Id streetId) const;
    virtual Measurement<double> streetMeanSpeed() const;
    virtual Measurement<double> streetMeanSpeed(double, bool) const;
    /// @brief Get the mean density of the streets in \f$m^{-1}\f$
    /// @return Measurement<double> The mean density of the streets and the standard deviation
    Measurement<double> streetMeanDensity(bool normalized = false) const;
    /// @brief Get the mean flow of the streets in \f$s^{-1}\f$
    /// @return Measurement<double> The mean flow of the streets and the standard deviation
    Measurement<double> streetMeanFlow() const;
    /// @brief Get the mean flow of the streets in \f$s^{-1}\f$
    /// @param threshold The density threshold to consider
    /// @param above If true, the function returns the mean flow of the streets with a density above the threshold, otherwise below
    /// @return Measurement<double> The mean flow of the streets and the standard deviation
    Measurement<double> streetMeanFlow(double threshold, bool above) const;

    /// @brief Save the street densities in csv format
    /// @param filename The name of the file (default is "{datetime}_{simulation_name}_street_densities.csv")
    /// @param normalized If true, the densities are normalized in [0, 1]
    void saveStreetDensities(std::string filename = std::string(),
                             bool normalized = true,
                             char const separator = ';') const;
    /// @brief Save the street input counts in csv format
    /// @param filename The name of the file
    /// @param reset If true, the input counts are cleared after the computation
    /// @details NOTE: counts are saved only if the street has a coil on it
    void saveCoilCounts(const std::string& filename,
                        bool reset = false,
                        char const separator = ';');
    /// @brief Save the travel data of the agents in csv format.
    /// @details The file contains the following columns:
    /// - time: the time of the simulation
    /// - distances: the travel distances of the agents
    /// - times: the travel times of the agents
    /// - speeds: the travel speeds of the agents
    /// @param filename The name of the file (default is "{datetime}_{simulation_name}_travel_data.csv")
    /// @param reset If true, the travel speeds are cleared after the computation
    void saveTravelData(std::string filename = std::string(), bool reset = false);
    /// @brief Save the main macroscopic observables in csv format
    /// @param filename The name of the file (default is "{datetime}_{simulation_name}_macroscopic_observables.csv")
    /// @param separator The separator character (default is ';')
    /// @details The file contains the following columns:
    /// - time: the time of the simulation
    /// - n_ghost_agents: the number of agents waiting to be inserted in the simulation
    /// - n_agents: the number of agents currently in the simulation
    /// - mean_speed - mean_speed_std (km/h): the mean speed of the agents
    /// - mean_density - mean_density_std (veh/km): the mean density of the streets
    /// - mean_flow - mean_flow_std (veh/h): the mean flow of the streets
    /// - mean_traveltime - mean_traveltime_std (min): the mean travel time of the agents
    /// - mean_traveldistance - mean_traveldistance_err (km): the mean travel distance of the agents
    /// - mean_travelspeed - mean_travelspeed_std (km/h): the mean travel speed of the agents
    ///
    /// NOTE: the mean density is normalized in [0, 1] and reset is true for all observables which have such parameter
    void saveMacroscopicObservables(std::string filename = std::string(),
                                    char const separator = ';');
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  RoadDynamics<delay_t>::RoadDynamics(RoadNetwork& graph,
                                      bool useCache,
                                      std::optional<unsigned int> seed,
                                      PathWeight const weightFunction,
                                      std::optional<double> weightTreshold)
      : Dynamics<RoadNetwork>(graph, seed),
        m_nAgents{0},
        m_previousOptimizationTime{0},
        m_errorProbability{std::nullopt},
        m_passageProbability{std::nullopt},
        m_maxTravelDistance{std::numeric_limits<double>::max()},
        m_maxTravelTime{std::numeric_limits<std::time_t>::max()},
        m_timeToleranceFactor{std::nullopt},
        m_bCacheEnabled{useCache},
        m_forcePriorities{false} {
    this->setWeightFunction(weightFunction, weightTreshold);
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(g_cacheFolder)) {
        std::filesystem::create_directory(g_cacheFolder);
      }
      spdlog::info("Cache enabled (default folder is {})", g_cacheFolder);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      m_nodeIndices.push_back(nodeId);
    }
    for (auto const& [nodeId, weight] : this->m_destinationNodes) {
      m_itineraries.emplace(nodeId, std::make_unique<Itinerary>(nodeId, nodeId));
    }
    // updatePaths();
    std::for_each(
        this->graph().edges().cbegin(),
        this->graph().edges().cend(),
        [this](auto const& pair) {
          auto const& pEdge{pair.second};
          auto const edgeId{pair.first};
          // fill turn mapping as [pair.first, [left street Id, straight street Id, right street Id, U self street Id]]
          m_turnMapping.emplace(edgeId, std::array<long, 4>{-1, -1, -1, -1});
          // Turn mappings
          const auto& srcNodeId = pEdge->target();
          for (auto const& outEdgeId : this->graph().node(srcNodeId)->outgoingEdges()) {
            auto const& pStreet{this->graph().edge(outEdgeId)};
            auto const previousStreetId = pStreet->id();
            auto const& delta{pEdge->deltaAngle(pStreet->angle())};
            if (std::abs(delta) < std::numbers::pi) {
              if (delta < 0.) {
                m_turnMapping[edgeId][dsf::Direction::RIGHT] = previousStreetId;  // right
              } else if (delta > 0.) {
                m_turnMapping[edgeId][dsf::Direction::LEFT] = previousStreetId;  // left
              } else {
                m_turnMapping[edgeId][dsf::Direction::STRAIGHT] =
                    previousStreetId;  // straight
              }
            } else {
              m_turnMapping[edgeId][dsf::Direction::UTURN] = previousStreetId;  // U
            }
          }
        });
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_updatePath(std::unique_ptr<Itinerary> const& pItinerary) {
    if (m_bCacheEnabled) {
      auto const& file = std::format("{}{}.ity", g_cacheFolder, pItinerary->id());
      if (std::filesystem::exists(file)) {
        pItinerary->load(file);
        spdlog::debug("Loaded cached path for itinerary {}", pItinerary->id());
        return;
      }
    }
    auto const oldSize{pItinerary->path().size()};

    auto const& path{this->graph().allPathsTo(
        pItinerary->destination(), m_weightFunction, m_weightTreshold)};
    if (path.empty()) {
      throw std::runtime_error(
          std::format("No path found for itinerary {} with destination node {}",
                      pItinerary->id(),
                      pItinerary->destination()));
    }
    pItinerary->setPath(path);
    auto const newSize{pItinerary->path().size()};
    if (oldSize > 0 && newSize != oldSize) {
      spdlog::warn("Path for itinerary {} changed size from {} to {}",
                   pItinerary->id(),
                   oldSize,
                   newSize);
    }
    if (m_bCacheEnabled) {
      pItinerary->save(std::format("{}{}.ity", g_cacheFolder, pItinerary->id()));
      spdlog::debug("Saved path in cache for itinerary {}", pItinerary->id());
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Id RoadDynamics<delay_t>::m_nextStreetId(std::unique_ptr<Agent> const& pAgent,
                                           Id nodeId,
                                           std::optional<Id> streetId) {
    // Get outgoing edges directly - avoid storing targets separately
    const auto& outgoingEdges = this->graph().node(nodeId)->outgoingEdges();
    std::vector<Id> possibleEdgeIds;
    possibleEdgeIds.reserve(outgoingEdges.size());  // Pre-allocate to avoid reallocations

    // Build forbidden target nodes set efficiently
    std::unordered_set<Id> forbiddenTargetNodes;
    if (streetId.has_value()) {
      auto const& pStreet{this->graph().edge(*streetId)};
      const auto& forbiddenTurns = pStreet->forbiddenTurns();
      forbiddenTargetNodes.insert(forbiddenTurns.begin(), forbiddenTurns.end());

      // Avoid U-TURNS, if possible
      if (!(this->graph().node(nodeId)->isRoundabout()) &&
          (outgoingEdges.size() > forbiddenTurns.size() + 1)) {
        auto const& pOppositeStreet{
            this->graph().street(pStreet->target(), pStreet->source())};
        if (pOppositeStreet) {
          forbiddenTargetNodes.insert(pOppositeStreet->get()->id());
        }
      }
    }

    // Log forbidden turns if any
    if (!forbiddenTargetNodes.empty()) {
      spdlog::debug("Excluding {} forbidden turns", forbiddenTargetNodes.size());
    }

    // For non-random agents, get allowed targets from itinerary
    std::unordered_set<Id> allowedTargets;
    bool hasItineraryConstraints = false;

    if (!pAgent->isRandom() && !this->itineraries().empty()) {
      std::uniform_real_distribution<double> uniformDist{0., 1.};
      if (!(m_errorProbability.has_value() &&
            uniformDist(this->m_generator) < m_errorProbability)) {
        const auto& it = this->itineraries().at(pAgent->itineraryId());
        if (it->destination() != nodeId) {
          try {
            const auto pathTargets = it->path().at(nodeId);
            allowedTargets.insert(pathTargets.begin(), pathTargets.end());
            hasItineraryConstraints = true;

            // Remove forbidden nodes from allowed targets
            for (const auto& forbiddenNodeId : forbiddenTargetNodes) {
              allowedTargets.erase(forbiddenNodeId);
            }
            // Catch unordered_map::at exceptions
          } catch (const std::out_of_range&) {
            throw std::runtime_error(std::format(
                "No path from node {} to destination {}", nodeId, it->destination()));
          }
        }
      }
    }

    // Single pass through outgoing edges with efficient filtering
    for (const auto outEdgeId : outgoingEdges) {
      const Id targetNode = this->graph().edge(outEdgeId)->target();

      // Skip if target is forbidden
      if (forbiddenTargetNodes.count(targetNode)) {
        continue;
      }

      // For non-random agents with itinerary constraints
      if (hasItineraryConstraints) {
        if (allowedTargets.count(targetNode)) {
          possibleEdgeIds.push_back(outEdgeId);
        }
      } else {
        // For random agents or when no itinerary constraints apply
        possibleEdgeIds.push_back(outEdgeId);
      }
    }

    if (possibleEdgeIds.empty()) {
      throw std::runtime_error(
          std::format("No possible moves for agent {} at node {}.", *pAgent, nodeId));
    }

    if (possibleEdgeIds.size() == 1) {
      return possibleEdgeIds[0];
    }

    std::uniform_int_distribution<Size> moveDist{
        0, static_cast<Size>(possibleEdgeIds.size() - 1)};
    return possibleEdgeIds[moveDist(this->m_generator)];
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveStreet(const std::unique_ptr<Street>& pStreet,
                                             bool reinsert_agents) {
    auto const nLanes = pStreet->nLanes();
    while (!pStreet->movingAgents().empty()) {
      auto const& pAgent{pStreet->movingAgents().top()};
      if (pAgent->freeTime() < this->time_step()) {
        break;
      }
      pAgent->setSpeed(0.);
      bool bArrived{false};
      if (!pAgent->isRandom()) {
        if (this->itineraries().at(pAgent->itineraryId())->destination() ==
            pStreet->target()) {
          pAgent->updateItinerary();
        }
        if (this->itineraries().at(pAgent->itineraryId())->destination() ==
            pStreet->target()) {
          bArrived = true;
        }
      }
      if (bArrived) {
        std::uniform_int_distribution<size_t> laneDist{0,
                                                       static_cast<size_t>(nLanes - 1)};
        pStreet->enqueue(laneDist(this->m_generator));
        continue;
      }
      auto const nextStreetId =
          this->m_nextStreetId(pAgent, pStreet->target(), pStreet->id());
      auto const& pNextStreet{this->graph().edge(nextStreetId)};
      pAgent->setNextStreetId(nextStreetId);
      if (nLanes == 1) {
        pStreet->enqueue(0);
        continue;
      }
      auto const direction{pNextStreet->turnDirection(pStreet->angle())};
      switch (direction) {
        case Direction::UTURN:
        case Direction::LEFT:
          pStreet->enqueue(nLanes - 1);
          break;
        case Direction::RIGHT:
          pStreet->enqueue(0);
          break;
        default:
          std::vector<double> weights;
          for (auto const& queue : pStreet->exitQueues()) {
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
          std::discrete_distribution<size_t> laneDist{weights.begin(), weights.end()};
          pStreet->enqueue(laneDist(this->m_generator));
      }
    }
    auto const& transportCapacity{pStreet->transportCapacity()};
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    for (auto i = 0; i < std::ceil(transportCapacity); ++i) {
      if (pStreet->isStochastic() &&
          uniformDist(this->m_generator) >
              dynamic_cast<StochasticStreet&>(*pStreet).flowRate()) {
        spdlog::debug("Skipping due to flow rate {:.2f} < random value",
                      dynamic_cast<StochasticStreet&>(*pStreet).flowRate());
        continue;
      }
      if (i == std::ceil(transportCapacity) - 1) {
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          spdlog::debug("Skipping due to fractional capacity {:.2f} < random value",
                        fractional);
          continue;
        }
      }
      for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
        if (pStreet->queue(queueIndex).empty()) {
          continue;
        }
        // Logger::debug("Taking temp agent");
        auto const& pAgentTemp{pStreet->queue(queueIndex).front()};
        if (pAgentTemp->freeTime() > this->time_step()) {
          spdlog::debug("Skipping due to time {} < free time {}",
                        this->time_step(),
                        pAgentTemp->freeTime());
          continue;
        }

        if (m_timeToleranceFactor.has_value()) {
          auto const timeDiff{this->time_step() - pAgentTemp->freeTime()};
          auto const timeTolerance{m_timeToleranceFactor.value() *
                                   std::ceil(pStreet->length() / pStreet->maxSpeed())};
          if (timeDiff > timeTolerance) {
            spdlog::warn(
                "Time-step {} - {} currently on {} ({} turn - Traffic Light? {}), "
                "has been still for more than {} seconds ({} seconds). Killing it.",
                this->time_step(),
                *pAgentTemp,
                *pStreet,
                directionToString.at(pStreet->laneMapping().at(queueIndex)),
                this->graph().node(pStreet->target())->isTrafficLight(),
                timeTolerance,
                timeDiff);
            // Kill the agent
            auto pAgent{pStreet->dequeue(queueIndex)};
            continue;
          }
        }
        pAgentTemp->setSpeed(0.);
        const auto& destinationNode{this->graph().node(pStreet->target())};
        if (destinationNode->isFull()) {
          spdlog::debug("Skipping due to full destination node {}", *destinationNode);
          continue;
        }
        if (destinationNode->isTrafficLight()) {
          auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
          auto const direction{pStreet->laneMapping().at(queueIndex)};
          if (!tl.isGreen(pStreet->id(), direction)) {
            spdlog::debug("Skipping due to red light on street {} and direction {}",
                          pStreet->id(),
                          directionToString.at(direction));
            continue;
          }
          spdlog::debug("Green light on street {} and direction {}",
                        pStreet->id(),
                        directionToString.at(direction));
        } else if (destinationNode->isIntersection() &&
                   pAgentTemp->nextStreetId().has_value()) {
          auto& intersection = static_cast<Intersection&>(*destinationNode);
          bool bCanPass{true};
          if (!intersection.streetPriorities().empty()) {
            spdlog::debug("Checking priorities for street {} -> {}",
                          pStreet->source(),
                          pStreet->target());
            auto const& thisDirection{this->graph()
                                          .edge(pAgentTemp->nextStreetId().value())
                                          ->turnDirection(pStreet->angle())};
            if (!intersection.streetPriorities().contains(pStreet->id())) {
              // I have to check if the agent has right of way
              auto const& inNeighbours{destinationNode->ingoingEdges()};
              for (auto const& inEdgeId : inNeighbours) {
                auto const& pStreetTemp{this->graph().edge(inEdgeId)};
                if (pStreetTemp->id() == pStreet->id()) {
                  continue;
                }
                if (pStreetTemp->nExitingAgents() == 0) {
                  continue;
                }
                if (intersection.streetPriorities().contains(pStreetTemp->id())) {
                  spdlog::debug(
                      "Skipping agent emission from street {} -> {} due to right of way.",
                      pStreet->source(),
                      pStreet->target());
                  bCanPass = false;
                  break;
                } else if (thisDirection >= Direction::LEFT) {
                  // Check if the agent has right of way using direction
                  // The problem arises only when you have to turn left
                  for (auto i{0}; i < pStreetTemp->nLanes(); ++i) {
                    // check queue is not empty and take the top agent
                    if (pStreetTemp->queue(i).empty()) {
                      continue;
                    }
                    auto const& pAgentTemp2{pStreetTemp->queue(i).front()};
                    if (!pAgentTemp2->nextStreetId().has_value()) {
                      continue;
                    }
                    auto const& otherDirection{
                        this->graph()
                            .edge(pAgentTemp2->nextStreetId().value())
                            ->turnDirection(this->graph()
                                                .edge(pAgentTemp2->streetId().value())
                                                ->angle())};
                    if (otherDirection < Direction::LEFT) {
                      spdlog::debug(
                          "Skipping agent emission from street {} -> {} due to right of "
                          "way with other agents.",
                          pStreet->source(),
                          pStreet->target());
                      bCanPass = false;
                      break;
                    }
                  }
                }
              }
            } else if (thisDirection >= Direction::LEFT) {
              for (auto const& streetId : intersection.streetPriorities()) {
                if (streetId == pStreet->id()) {
                  continue;
                }
                auto const& pStreetTemp{this->graph().edge(streetId)};
                for (auto i{0}; i < pStreetTemp->nLanes(); ++i) {
                  // check queue is not empty and take the top agent
                  if (pStreetTemp->queue(i).empty()) {
                    continue;
                  }
                  auto const& pAgentTemp2{pStreetTemp->queue(i).front()};
                  if (!pAgentTemp2->streetId().has_value()) {
                    continue;
                  }
                  auto const& otherDirection{
                      this->graph()
                          .edge(pAgentTemp2->nextStreetId().value())
                          ->turnDirection(this->graph()
                                              .edge(pAgentTemp2->streetId().value())
                                              ->angle())};
                  if (otherDirection < thisDirection) {
                    spdlog::debug(
                        "Skipping agent emission from street {} -> {} due to right of "
                        "way with other agents.",
                        pStreet->source(),
                        pStreet->target());
                    bCanPass = false;
                    break;
                  }
                }
              }
            }
          }
          if (!bCanPass) {
            spdlog::debug(
                "Skipping agent emission from street {} -> {} due to right of way",
                pStreet->source(),
                pStreet->target());
            continue;
          }
        }
        bool bArrived{false};
        if (!(uniformDist(this->m_generator) <
              m_passageProbability.value_or(std::numeric_limits<double>::max()))) {
          if (pAgentTemp->isRandom()) {
            bArrived = true;
          } else {
            spdlog::debug(
                "Skipping agent emission from street {} -> {} due to passage "
                "probability",
                pStreet->source(),
                pStreet->target());
            continue;
          }
        }
        if (!pAgentTemp->isRandom()) {
          if (destinationNode->id() ==
              this->itineraries().at(pAgentTemp->itineraryId())->destination()) {
            bArrived = true;
            spdlog::debug("Agent {} has arrived at destination node {}",
                          pAgentTemp->id(),
                          destinationNode->id());
          }
        } else {
          if (pAgentTemp->distance() >= m_maxTravelDistance) {
            bArrived = true;
          }
          if (!bArrived &&
              (this->time_step() - pAgentTemp->spawnTime() >= m_maxTravelTime)) {
            bArrived = true;
          }
        }
        if (bArrived) {
          auto pAgent{pStreet->dequeue(queueIndex)};
          spdlog::debug(
              "{} has arrived at destination node {}", *pAgent, destinationNode->id());
          m_travelDTs.push_back(
              {pAgent->distance(),
               static_cast<double>(this->time_step() - pAgent->spawnTime())});
          --m_nAgents;
          if (reinsert_agents) {
            // reset Agent's values
            pAgent->reset(this->time_step());
            this->addAgent(std::move(pAgent));
          }
          continue;
        }
        if (!pAgentTemp->streetId().has_value()) {
          spdlog::error("{} has no street id", *pAgentTemp);
        }
        auto const& nextStreet{this->graph().edge(pAgentTemp->nextStreetId().value())};
        if (nextStreet->isFull()) {
          spdlog::debug(
              "Skipping agent emission from street {} -> {} due to full "
              "next street: {}",
              pStreet->source(),
              pStreet->target(),
              *nextStreet);
          continue;
        }
        auto pAgent{pStreet->dequeue(queueIndex)};
        assert(destinationNode->id() == nextStreet->source());
        if (destinationNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
          auto const delta{nextStreet->deltaAngle(pStreet->angle())};
          intersection.addAgent(delta, std::move(pAgent));
        } else if (destinationNode->isRoundabout()) {
          auto& roundabout = dynamic_cast<Roundabout&>(*destinationNode);
          roundabout.enqueue(std::move(pAgent));
        }
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveNode(const std::unique_ptr<RoadJunction>& pNode) {
    auto const transportCapacity{pNode->transportCapacity()};
    for (auto i{0}; i < std::ceil(transportCapacity); ++i) {
      if (i == std::ceil(transportCapacity) - 1) {
        std::uniform_real_distribution<double> uniformDist{0., 1.};
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          spdlog::debug("Skipping dequeue from node {} due to transport capacity {}",
                        pNode->id(),
                        transportCapacity);
          return;
        }
      }
      if (pNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pNode);
        if (intersection.agents().empty()) {
          return;
        }
        for (auto it{intersection.agents().begin()}; it != intersection.agents().end();) {
          auto& pAgent{it->second};
          auto const& nextStreet{this->graph().edge(pAgent->nextStreetId().value())};
          if (nextStreet->isFull()) {
            spdlog::debug("Next street is full: {}", *nextStreet);
            if (m_forcePriorities) {
              spdlog::debug("Forcing priority from {} on {}", *pNode, *nextStreet);
              return;
            }
            ++it;
            continue;
          }
          if (!m_turnCounts.empty() && pAgent->streetId().has_value()) {
            ++m_turnCounts[*(pAgent->streetId())][nextStreet->id()];
          }
          pAgent->setStreetId();
          this->setAgentSpeed(pAgent);
          pAgent->setFreeTime(this->time_step() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
          spdlog::debug(
              "{} at time {} has been dequeued from intersection {} and "
              "enqueued on street {} with free time {}.",
              *pAgent,
              this->time_step(),
              pNode->id(),
              nextStreet->id(),
              pAgent->freeTime());
          nextStreet->addAgent(std::move(pAgent));
          it = intersection.agents().erase(it);
          break;
        }
      } else if (pNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
        if (roundabout.agents().empty()) {
          return;
        }
        auto const& pAgentTemp{roundabout.agents().front()};
        auto const& nextStreet{this->graph().edge(pAgentTemp->nextStreetId().value())};
        if (!(nextStreet->isFull())) {
          if (!m_turnCounts.empty() && pAgentTemp->streetId().has_value()) {
            ++m_turnCounts[*(pAgentTemp->streetId())][nextStreet->id()];
          }
          auto pAgent{roundabout.dequeue()};
          pAgent->setStreetId();
          this->setAgentSpeed(pAgent);
          pAgent->setFreeTime(this->time_step() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
          spdlog::debug(
              "An agent at time {} has been dequeued from roundabout {} and "
              "enqueued on street {} with free time {}: {}",
              this->time_step(),
              pNode->id(),
              nextStreet->id(),
              pAgent->freeTime(),
              *pAgent);
          nextStreet->addAgent(std::move(pAgent));
        } else {
          return;
        }
      }
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveAgents() {
    std::uniform_int_distribution<Id> nodeDist{
        0, static_cast<Id>(this->graph().nNodes() - 1)};
    spdlog::debug("Processing {} agents", m_agents.size());
    for (auto itAgent{m_agents.begin()}; itAgent != m_agents.end();) {
      auto& pAgent{*itAgent};
      if (!pAgent->srcNodeId().has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        pAgent->setSrcNodeId(nodeIt->second->id());
      }
      auto const& pSourceNode{this->graph().node(*(pAgent->srcNodeId()))};
      if (pSourceNode->isFull()) {
        spdlog::debug("Skipping {} due to full source {}", *pAgent, *pSourceNode);
        ++itAgent;
        continue;
      }
      if (!pAgent->nextStreetId().has_value()) {
        spdlog::debug("No next street id, generating a random one");
        pAgent->setNextStreetId(
            this->m_nextStreetId(pAgent, pSourceNode->id(), pAgent->streetId()));
      }
      // spdlog::debug("Checking next street {}", pAgent->nextStreetId().value());
      auto const& nextStreet{
          this->graph().edge(pAgent->nextStreetId().value())};  // next street
      if (nextStreet->isFull()) {
        ++itAgent;
        spdlog::debug("Skipping {} due to full input {}", *pAgent, *nextStreet);
        continue;
      }
      // spdlog::debug("Adding agent on the source node");
      if (pSourceNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pSourceNode);
        intersection.addAgent(0., std::move(pAgent));
      } else if (pSourceNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pSourceNode);
        roundabout.enqueue(std::move(pAgent));
      }
      itAgent = m_agents.erase(itAgent);
    }
    spdlog::debug("There are {} agents left in the list.", m_agents.size());
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setErrorProbability(double errorProbability) {
    if (errorProbability < 0. || errorProbability > 1.) {
      throw std::invalid_argument(
          std::format("The error probability ({}) must be in [0, 1]", errorProbability));
    }
    m_errorProbability = errorProbability;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setPassageProbability(double passageProbability) {
    if (passageProbability < 0. || passageProbability > 1.) {
      throw std::invalid_argument(std::format(
          "The passage probability ({}) must be in [0, 1]", passageProbability));
    }
    m_passageProbability = passageProbability;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::killStagnantAgents(double timeToleranceFactor) {
    if (timeToleranceFactor <= 0.) {
      throw std::invalid_argument(std::format(
          "The time tolerance factor ({}) must be positive", timeToleranceFactor));
    }
    m_timeToleranceFactor = timeToleranceFactor;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setWeightFunction(PathWeight const pathWeight,
                                                std::optional<double> weightTreshold) {
    switch (pathWeight) {
      case PathWeight::LENGTH:
        m_weightFunction = [](std::unique_ptr<Street> const& pStreet) {
          return pStreet->length();
        };
        m_weightTreshold = weightTreshold.value_or(1.);
        break;
      case PathWeight::TRAVELTIME:
        m_weightFunction = [this](std::unique_ptr<Street> const& pStreet) {
          return this->m_streetEstimatedTravelTime(pStreet);
        };
        m_weightTreshold = weightTreshold.value_or(0.0069);
        break;
      case PathWeight::WEIGHT:
        m_weightFunction = [](std::unique_ptr<Street> const& pStreet) {
          return pStreet->weight();
        };
        m_weightTreshold = weightTreshold.value_or(1.);
        break;
      default:
        spdlog::error("Invalid weight function. Defaulting to traveltime");
        m_weightFunction = [this](std::unique_ptr<Street> const& pStreet) {
          return this->m_streetEstimatedTravelTime(pStreet);
        };
        m_weightTreshold = weightTreshold.value_or(0.0069);
        break;
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setOriginNodes(
      std::unordered_map<Id, double> const& originNodes) {
    m_originNodes.clear();
    m_originNodes.reserve(originNodes.size());
    auto const sumWeights = std::accumulate(
        originNodes.begin(), originNodes.end(), 0., [](double sum, auto const& pair) {
          return sum + pair.second;
        });
    if (sumWeights == 1.) {
      m_originNodes = originNodes;
      return;
    }
    for (auto const& [nodeId, weight] : originNodes) {
      m_originNodes[nodeId] = weight / sumWeights;
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setDestinationNodes(
      std::unordered_map<Id, double> const& destinationNodes) {
    m_destinationNodes.clear();
    m_destinationNodes.reserve(destinationNodes.size());
    auto sumWeights{0.};
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &sumWeights](auto const& pair) -> void {
                    sumWeights += pair.second;
                    if (this->itineraries().contains(pair.first)) {
                      return;
                    }
                    this->addItinerary(pair.first, pair.first);
                  });
    if (sumWeights == 1.) {
      m_destinationNodes = destinationNodes;
      return;
    }
    for (auto const& [nodeId, weight] : destinationNodes) {
      m_destinationNodes[nodeId] = weight / sumWeights;
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::initTurnCounts() {
    if (!m_turnCounts.empty()) {
      throw std::runtime_error("Turn counts have already been initialized.");
    }
    for (auto const& [edgeId, pEdge] : this->graph().edges()) {
      auto const& pTargetNode{this->graph().node(pEdge->target())};
      for (auto const& nextEdgeId : pTargetNode->outgoingEdges()) {
        spdlog::debug("Initializing turn count for edge {} -> {}", edgeId, nextEdgeId);
        m_turnCounts[edgeId][nextEdgeId] = 0;
      }
    }
  }
  // You may wonder why not just use one function...
  // Never trust the user!
  // Jokes aside, the init is necessary because it allocates the memory for the first time and
  // turn counts are not incremented if the map is empty for performance reasons.
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::resetTurnCounts() {
    if (m_turnCounts.empty()) {
      throw std::runtime_error("Turn counts have not been initialized.");
    }
    for (auto const& [edgeId, pEdge] : this->graph().edges()) {
      auto const& pTargetNode{this->graph().node(pEdge->target())};
      for (auto const& nextEdgeId : pTargetNode->outgoingEdges()) {
        m_turnCounts[edgeId][nextEdgeId] = 0;
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::setDestinationNodes(
      std::initializer_list<Id> destinationNodes) {
    auto const numNodes{destinationNodes.size()};
    m_destinationNodes.clear();
    m_destinationNodes.reserve(numNodes);
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &numNodes](auto const& nodeId) -> void {
                    this->m_destinationNodes[nodeId] = 1. / numNodes;
                    if (this->itineraries().contains(nodeId)) {
                      return;
                    }
                    this->addItinerary(nodeId, nodeId);
                  });
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_convertible_v<typename TContainer::value_type, Id>)
  void RoadDynamics<delay_t>::setDestinationNodes(TContainer const& destinationNodes) {
    auto const numNodes{destinationNodes.size()};
    m_destinationNodes.clear();
    m_destinationNodes.reserve(numNodes);
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &numNodes](auto const& nodeId) -> void {
                    this->m_destinationNodes[nodeId] = 1. / numNodes;
                    if (this->itineraries().contains(nodeId)) {
                      return;
                    }
                    this->addItinerary(nodeId, nodeId);
                  });
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::updatePaths() {
    spdlog::debug("Init updating paths...");
    tbb::parallel_for_each(
        this->itineraries().cbegin(),
        this->itineraries().cend(),
        [this](auto const& pair) -> void { this->m_updatePath(pair.second); });
    spdlog::debug("End updating paths.");
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgentsUniformly(Size nAgents,
                                                 std::optional<Id> optItineraryId) {
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::warn(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsUniformly().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    if (optItineraryId.has_value() && !this->itineraries().contains(*optItineraryId)) {
      throw std::invalid_argument(
          std::format("No itineraries available. Cannot add agents with itinerary id {}",
                      optItineraryId.value()));
    }
    bool const bRandomItinerary{!optItineraryId.has_value() &&
                                !this->itineraries().empty()};
    std::optional<Id> itineraryId{std::nullopt};
    std::uniform_int_distribution<Size> itineraryDist{
        0, static_cast<Size>(this->itineraries().size() - 1)};
    std::uniform_int_distribution<Size> streetDist{
        0, static_cast<Size>(this->graph().nEdges() - 1)};
    if (this->nAgents() + nAgents > this->graph().capacity()) {
      throw std::overflow_error(std::format(
          "Cannot add {} agents. The graph has currently {} with a maximum capacity of "
          "{}.",
          nAgents,
          this->nAgents(),
          this->graph().capacity()));
    }
    for (Size i{0}; i < nAgents; ++i) {
      if (bRandomItinerary) {
        auto itineraryIt{this->itineraries().cbegin()};
        std::advance(itineraryIt, itineraryDist(this->m_generator));
        itineraryId = itineraryIt->first;
      }
      auto streetIt = this->graph().edges().begin();
      while (true) {
        Size step = streetDist(this->m_generator);
        std::advance(streetIt, step);
        if (!(streetIt->second->isFull())) {
          break;
        }
        streetIt = this->graph().edges().begin();
      }
      auto const& street{streetIt->second};
      this->addAgent(
          std::make_unique<Agent>(this->time_step(), itineraryId, street->source()));
      auto& pAgent{this->m_agents.back()};
      pAgent->setStreetId(street->id());
      this->setAgentSpeed(pAgent);
      pAgent->setFreeTime(this->time_step() +
                          std::ceil(street->length() / pAgent->speed()));
      street->addAgent(std::move(pAgent));
      this->m_agents.pop_back();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
             std::is_same_v<TContainer, std::map<Id, double>>)
  void RoadDynamics<delay_t>::addAgentsRandomly(Size nAgents,
                                                const TContainer& src_weights,
                                                const TContainer& dst_weights) {
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::warn(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsRandomly().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    auto const& nSources{src_weights.size()};
    auto const& nDestinations{dst_weights.size()};
    spdlog::debug("Init addAgentsRandomly for {} agents from {} nodes to {} nodes.",
                  nAgents,
                  nSources,
                  nDestinations);
    if (nSources == 1 && nDestinations == 1 &&
        src_weights.begin()->first == dst_weights.begin()->first) {
      throw std::invalid_argument(
          std::format("The only source node {} is also the only destination node.",
                      src_weights.begin()->first));
    }
    auto const srcSum{std::accumulate(
        src_weights.begin(),
        src_weights.end(),
        0.,
        [](double sum, const std::pair<Id, double>& p) {
          if (p.second < 0.) {
            throw std::invalid_argument(std::format(
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
            throw std::invalid_argument(std::format(
                "Negative weight ({}) for destination node {}.", p.second, p.first));
          }
          return sum + p.second;
        })};
    std::uniform_int_distribution<size_t> nodeDist{
        0, static_cast<size_t>(this->graph().nNodes() - 1)};
    std::uniform_real_distribution<double> srcUniformDist{0., srcSum};
    std::uniform_real_distribution<double> dstUniformDist{0., dstSum};
    spdlog::debug("Adding {} agents at time {}.", nAgents, this->time_step());
    while (nAgents > 0) {
      std::optional<Id> srcId{std::nullopt}, dstId{std::nullopt};

      // Select source using weighted random selection
      if (nSources == 1) {
        srcId = src_weights.begin()->first;
      } else {
        double dRand = srcUniformDist(this->m_generator);
        double sum = 0.;
        for (const auto& [id, weight] : src_weights) {
          sum += weight;
          if (dRand < sum) {
            srcId = id;
            break;
          }
        }
      }

      // Select destination using weighted random selection
      if (nDestinations == 1) {
        dstId = dst_weights.begin()->first;
      } else {
        double dRand = dstUniformDist(this->m_generator);
        double sum = 0.;
        for (const auto& [id, weight] : dst_weights) {
          sum += weight;
          if (dRand < sum) {
            dstId = id;
            break;
          }
        }
      }

      // Fallback to random nodes if selection failed
      if (!srcId.has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        srcId = nodeIt->first;
      }
      if (!dstId.has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        dstId = nodeIt->first;
      }

      // Find the itinerary with the given destination
      auto itineraryIt{std::find_if(this->itineraries().cbegin(),
                                    this->itineraries().cend(),
                                    [dstId](const auto& itinerary) {
                                      return itinerary.second->destination() == *dstId;
                                    })};
      if (itineraryIt == this->itineraries().cend()) {
        spdlog::error("Itinerary with destination {} not found. Skipping agent.", *dstId);
        --nAgents;
        continue;
      }

      // Check if destination is reachable from source
      auto const& itinerary = itineraryIt->second;
      if (!itinerary->path().contains(*srcId)) {
        spdlog::warn("Destination {} not reachable from source {}. Skipping agent.",
                     *dstId,
                     *srcId);
        --nAgents;
        continue;
      }

      this->addAgent(itineraryIt->first, *srcId);
      --nAgents;
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgentsRandomly(Size nAgents) {
    addAgentsRandomly(nAgents, this->m_originNodes, this->m_destinationNodes);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgent(std::unique_ptr<Agent> pAgent) {
    m_agents.push_back(std::move(pAgent));
    ++m_nAgents;
    spdlog::debug("Added {}", *m_agents.back());
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
  void RoadDynamics<delay_t>::addAgent(TArgs&&... args) {
    addAgent(std::make_unique<Agent>(this->time_step(), std::forward<TArgs>(args)...));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, std::time_t, TArgs...>)
  void RoadDynamics<delay_t>::addAgents(Size nAgents, TArgs&&... args) {
    for (size_t i{0}; i < nAgents; ++i) {
      addAgent(std::make_unique<Agent>(this->time_step(), std::forward<TArgs>(args)...));
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Itinerary, TArgs...>)
  void RoadDynamics<delay_t>::addItinerary(TArgs&&... args) {
    addItinerary(std::make_unique<Itinerary>(std::forward<TArgs>(args)...));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addItinerary(std::unique_ptr<Itinerary> itinerary) {
    if (m_itineraries.contains(itinerary->id())) {
      throw std::invalid_argument(
          std::format("Itinerary with id {} already exists.", itinerary->id()));
    }
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::evolve(bool reinsert_agents) {
    spdlog::debug("Init evolve at time {}", this->time_step());
    // move the first agent of each street queue, if possible, putting it in the next node
    bool const bUpdateData = m_dataUpdatePeriod.has_value() &&
                             this->time_step() % m_dataUpdatePeriod.value() == 0;
    auto const numNodes{this->graph().nNodes()};

    const unsigned int concurrency = std::thread::hardware_concurrency();
    // Calculate a grain size to partition the nodes into roughly "concurrency" blocks
    const size_t grainSize = std::max(size_t(1), numNodes / concurrency);
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
              auto const& pNode = this->graph().node(m_nodeIndices[i]);
              for (auto const& inEdgeId : pNode->ingoingEdges()) {
                auto const& pStreet{this->graph().edge(inEdgeId)};
                if (bUpdateData && pNode->isTrafficLight()) {
                  if (!m_queuesAtTrafficLights.contains(inEdgeId)) {
                    auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                    assert(!tl.cycles().empty());
                    for (auto const& [id, pair] : tl.cycles()) {
                      for (auto const& [direction, cycle] : pair) {
                        m_queuesAtTrafficLights[id].emplace(direction, 0.);
                      }
                    }
                  }
                  for (auto& [direction, value] : m_queuesAtTrafficLights.at(inEdgeId)) {
                    value += pStreet->nExitingAgents(direction, true);
                  }
                }
                m_evolveStreet(pStreet, reinsert_agents);
              }
            }
          });
    });
    spdlog::debug("Pre-nodes");
    // Move transport capacity agents from each node
    this->m_taskArena.execute([&] {
      tbb::parallel_for(tbb::blocked_range<size_t>(0, numNodes, grainSize),
                        [&](const tbb::blocked_range<size_t>& range) {
                          for (size_t i = range.begin(); i != range.end(); ++i) {
                            const auto& pNode = this->graph().node(m_nodeIndices[i]);
                            m_evolveNode(pNode);
                            if (pNode->isTrafficLight()) {
                              auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                              ++tl;
                            }
                          }
                        });
    });
    this->m_evolveAgents();
    // cycle over agents and update their times
    Dynamics<RoadNetwork>::m_evolve();
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_trafficlightSingleTailOptimizer(
      double const& beta, std::optional<std::ofstream>& logStream) {
    assert(beta >= 0. && beta <= 1.);
    if (logStream.has_value()) {
      *logStream << std::format(
          "Init Traffic Lights optimization (SINGLE TAIL) - Time {} - Alpha {}\n",
          this->time_step(),
          beta);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);

      auto const& inNeighbours{pNode->ingoingEdges()};

      // Default is RIGHTANDSTRAIGHT - LEFT phases for both priority and non-priority
      std::array<double, 2> inputPrioritySum{0., 0.}, inputNonPrioritySum{0., 0.};
      bool isPrioritySinglePhase{false}, isNonPrioritySinglePhase{false};

      for (const auto& streetId : inNeighbours) {
        if (tl.cycles().at(streetId).contains(Direction::ANY)) {
          tl.streetPriorities().contains(streetId) ? isPrioritySinglePhase = true
                                                   : isNonPrioritySinglePhase = true;
        }
      }
      if (isPrioritySinglePhase && logStream.has_value()) {
        *logStream << "\tFound a single phase for priority streets.\n";
      }
      if (isNonPrioritySinglePhase && logStream.has_value()) {
        *logStream << "\tFound a single phase for non-priority streets.\n";
      }

      for (const auto& streetId : inNeighbours) {
        for (auto const& [direction, tail] : m_queuesAtTrafficLights.at(streetId)) {
          if (tl.streetPriorities().contains(streetId)) {
            if (isPrioritySinglePhase) {
              inputPrioritySum[0] += tail;
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                inputPrioritySum[1] += tail;
              } else {
                inputPrioritySum[0] += tail;
              }
            }
          } else {
            if (isNonPrioritySinglePhase) {
              inputNonPrioritySum[0] += tail;
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                inputNonPrioritySum[1] += tail;
              } else {
                inputNonPrioritySum[0] += tail;
              }
            }
          }
        }
      }
      {
        // Sum normalization
        auto const sum{inputPrioritySum[0] + inputPrioritySum[1] +
                       inputNonPrioritySum[0] + inputNonPrioritySum[1]};
        if (sum == 0.) {
          continue;
        }
        inputPrioritySum[0] /= sum;
        inputPrioritySum[1] /= sum;
        inputNonPrioritySum[0] /= sum;
        inputNonPrioritySum[1] /= sum;

        // int const cycleTime{(1. - alpha) * tl.cycleTime()};

        inputPrioritySum[0] *= beta;
        inputPrioritySum[1] *= beta;
        inputNonPrioritySum[0] *= beta;
        inputNonPrioritySum[1] *= beta;
      }

      if (logStream.has_value()) {
        *logStream << std::format(
            "\tInput cycle queue ratios are {:.2f} {:.2f} - {:.2f} {:.2f}\n",
            inputPrioritySum[0],
            inputPrioritySum[1],
            inputNonPrioritySum[0],
            inputNonPrioritySum[1]);
      }

      tl.resetCycles();
      auto cycles{tl.cycles()};
      std::array<int, 4> n{0, 0, 0, 0};
      std::array<double, 4> greenTimes{0., 0., 0., 0.};

      for (auto const& [streetId, pair] : cycles) {
        for (auto const& [direction, cycle] : pair) {
          if (tl.streetPriorities().contains(streetId)) {
            if (isPrioritySinglePhase) {
              greenTimes[0] += cycle.greenTime();
              ++n[0];
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                greenTimes[1] += cycle.greenTime();
                ++n[1];
              } else {
                greenTimes[0] += cycle.greenTime();
                ++n[0];
              }
            }
          } else {
            if (isNonPrioritySinglePhase) {
              greenTimes[2] += cycle.greenTime();
              ++n[2];
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                greenTimes[3] += cycle.greenTime();
                ++n[3];
              } else {
                greenTimes[2] += cycle.greenTime();
                ++n[2];
              }
            }
          }
        }
      }

      if (logStream.has_value()) {
        *logStream << std::format("\tGreen times are {} {} - {} {}\n",
                                  greenTimes[0],
                                  greenTimes[1],
                                  greenTimes[2],
                                  greenTimes[3]);
      }

      for (auto i{0}; i < 4; ++i) {
        if (n[i] > 1) {
          greenTimes[i] /= n[i];
        }
      }

      {
        auto sum{0.};
        for (auto const& greenTime : greenTimes) {
          sum += greenTime;
        }
        if (sum == 0.) {
          continue;
        }
        for (auto& greenTime : greenTimes) {
          greenTime /= sum;
        }
      }
      for (auto& el : greenTimes) {
        el *= (1. - beta);
      }

      int inputPriorityR{static_cast<int>(
          std::floor((inputPrioritySum[0] + greenTimes[0]) * tl.cycleTime()))};
      int inputPriorityS{inputPriorityR};
      int inputPriorityL{static_cast<int>(
          std::floor((inputPrioritySum[1] + greenTimes[1]) * tl.cycleTime()))};

      int inputNonPriorityR{static_cast<int>(
          std::floor((inputNonPrioritySum[0] + greenTimes[2]) * tl.cycleTime()))};
      int inputNonPriorityS{inputNonPriorityR};
      int inputNonPriorityL{static_cast<int>(
          std::floor((inputNonPrioritySum[1] + greenTimes[3]) * tl.cycleTime()))};

      {
        // Adjust phases to have the sum equal to the cycle time
        // To do this, first add seconds to the priority streets, then to the
        // non-priority streets
        auto total{static_cast<Delay>(inputPriorityR + inputPriorityL +
                                      inputNonPriorityR + inputNonPriorityL)};
        size_t idx{0};
        while (total < tl.cycleTime()) {
          switch (idx % 4) {
            case 0:
              ++inputPriorityR;
              ++inputPriorityS;
              break;
            case 1:
              ++inputPriorityL;
              break;
            case 2:
              ++inputNonPriorityR;
              ++inputNonPriorityS;
              break;
            case 3:
              ++inputNonPriorityL;
              break;
          }
          ++idx;
          ++total;
        }
      }

      if (isPrioritySinglePhase) {
        inputPriorityR = 0;
        inputPriorityL = 0;
      }
      if (isNonPrioritySinglePhase) {
        inputNonPriorityR = 0;
        inputNonPriorityL = 0;
      }

      // Logger::info(std::format(
      //     "Cycle time: {} - Current sum: {}",
      //     tl.cycleTime(),
      //     inputPriorityRS + inputPriorityL + inputNonPriorityRS + inputNonPriorityL));
      assert(inputPriorityS + inputPriorityL + inputNonPriorityS + inputNonPriorityL ==
             tl.cycleTime());

      std::unordered_map<Direction, TrafficLightCycle> priorityCycles;
      priorityCycles.emplace(Direction::RIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityR), 0});
      priorityCycles.emplace(Direction::STRAIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityS), 0});
      priorityCycles.emplace(Direction::RIGHTANDSTRAIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityS), 0});
      priorityCycles.emplace(
          Direction::ANY,
          TrafficLightCycle{static_cast<Delay>(inputPriorityS + inputPriorityL), 0});
      priorityCycles.emplace(Direction::LEFT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityL),
                                               static_cast<Delay>(inputPriorityS)});

      std::unordered_map<Direction, TrafficLightCycle> nonPriorityCycles;
      nonPriorityCycles.emplace(
          Direction::RIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityR),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::STRAIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::RIGHTANDSTRAIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::ANY,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::LEFT,
          TrafficLightCycle{
              static_cast<Delay>(inputNonPriorityL),
              static_cast<Delay>(inputPriorityS + inputPriorityL + inputNonPriorityS)});
      nonPriorityCycles.emplace(
          Direction::LEFTANDSTRAIGHT,
          TrafficLightCycle{
              static_cast<Delay>(inputNonPriorityL + inputNonPriorityS),
              static_cast<Delay>(inputPriorityS + inputPriorityL + inputNonPriorityR)});

      std::vector<Id> streetIds;
      std::set<Id> forbiddenLeft;

      for (auto const& pair : cycles) {
        streetIds.push_back(pair.first);
      }
      for (auto const streetId : streetIds) {
        auto const& pStreet{this->graph().edge(streetId)};
        if (tl.streetPriorities().contains(streetId)) {
          for (auto& [dir, cycle] : cycles.at(streetId)) {
            if (isPrioritySinglePhase) {
              cycle = priorityCycles.at(Direction::STRAIGHT);
            } else {
              cycle = priorityCycles.at(dir);
            }
          }
          if (cycles.at(streetId).contains(Direction::RIGHT) &&
              cycles.at(streetId).contains(Direction::STRAIGHT)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputPriorityS + inputPriorityL), 0};
            // Logger::info(std::format("Free cycle (RIGHT) for {} -> {}: {} {}",
            //                          pStreet->source(),
            //                          pStreet->target(),
            //                          freecycle.greenTime(),
            //                          freecycle.phase()));
            cycles.at(streetId).at(Direction::RIGHT) = freecycle;
          }
        } else {
          for (auto& [dir, cycle] : cycles.at(streetId)) {
            if (isNonPrioritySinglePhase) {
              cycle = nonPriorityCycles.at(Direction::STRAIGHT);
            } else {
              cycle = nonPriorityCycles.at(dir);
            }
          }
          if (cycles.at(streetId).contains(Direction::RIGHT) &&
              cycles.at(streetId).contains(Direction::STRAIGHT)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                static_cast<Delay>(inputPriorityS + inputPriorityL)};
            // Logger::info(std::format("Free cycle (RIGHT) for {} -> {}: {} {}",
            //                          pStreet->source(),
            //                          pStreet->target(),
            //                          freecycle.greenTime(),
            //                          freecycle.phase()));
            cycles.at(streetId).at(Direction::RIGHT) = freecycle;
          }
        }
        bool found{false};
        for (auto const dir : pStreet->laneMapping()) {
          if (dir == Direction::LEFT || dir == Direction::LEFTANDSTRAIGHT ||
              dir == Direction::ANY) {
            found = true;
            break;
          }
        }
        if (!found) {
          forbiddenLeft.insert(streetId);
          // Logger::info(std::format("Street {} -> {} has forbidden left turn.",
          //                          pStreet->source(),
          //                          pStreet->target()));
        }
      }
      for (auto const forbiddenLeftStreetId : forbiddenLeft) {
        for (auto const streetId : streetIds) {
          if (streetId == forbiddenLeftStreetId) {
            continue;
          }
          if (tl.streetPriorities().contains(streetId) &&
              tl.streetPriorities().contains(forbiddenLeftStreetId)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputPriorityS + inputPriorityL), 0};
            for (auto& [direction, cycle] : cycles.at(streetId)) {
              if (direction == Direction::RIGHT || direction == Direction::STRAIGHT ||
                  direction == Direction::RIGHTANDSTRAIGHT) {
                auto const& pStreet{this->graph().edge(streetId)};
                if (logStream.has_value()) {
                  *logStream << std::format("\tFree cycle for {} -> {}: dir {} - {}\n",
                                            pStreet->source(),
                                            pStreet->target(),
                                            directionToString[direction],
                                            freecycle);
                }
                cycle = freecycle;
              }
            }
          } else if (!tl.streetPriorities().contains(streetId) &&
                     !tl.streetPriorities().contains(forbiddenLeftStreetId)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                static_cast<Delay>(inputPriorityS + inputPriorityL)};
            for (auto& [direction, cycle] : cycles.at(streetId)) {
              if (direction == Direction::RIGHT || direction == Direction::STRAIGHT ||
                  direction == Direction::RIGHTANDSTRAIGHT) {
                auto const& pStreet{this->graph().edge(streetId)};
                if (logStream.has_value()) {
                  *logStream << std::format("Free cycle ({}) for {} -> {}: {} {}\n",
                                            directionToString[direction],
                                            pStreet->source(),
                                            pStreet->target(),
                                            freecycle.greenTime(),
                                            freecycle.phase());
                }
                cycle = freecycle;
              }
            }
          }
        }
      }

      tl.setCycles(cycles);
      if (logStream.has_value()) {
        *logStream << std::format("\nNew cycles for {}", tl);
      }
    }
    if (logStream.has_value()) {
      *logStream << std::format("End Traffic Lights optimization - Time {}\n",
                                this->time_step());
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::optimizeTrafficLights(
      TrafficLightOptimization const optimizationType,
      const std::string& logFile,
      double const percentage,
      double const threshold) {
    std::optional<std::ofstream> logStream;
    if (!logFile.empty()) {
      logStream.emplace(logFile, std::ios::app);
      if (!logStream->is_open()) {
        spdlog::error("Could not open log file: {}", logFile);
      }
    }
    this->m_trafficlightSingleTailOptimizer(percentage, logStream);
    if (optimizationType == TrafficLightOptimization::DOUBLE_TAIL) {
      // Try to synchronize congested traffic lights
      std::unordered_map<Id, double> densities;
      for (auto const& [nodeId, pNode] : this->graph().nodes()) {
        if (!pNode->isTrafficLight()) {
          continue;
        }
        double density{0.}, n{0.};
        auto const& inNeighbours{pNode->ingoingEdges()};
        for (auto const& inEdgeId : inNeighbours) {
          auto const& pStreet{this->graph().edge(inEdgeId)};
          auto const& pSourceNode{this->graph().node(pStreet->source())};
          if (!pSourceNode->isTrafficLight()) {
            continue;
          }
          density += pStreet->density(true);
          ++n;
        }
        density /= n;
        densities[nodeId] = density;
      }
      // Sort densities map from big to small values
      std::vector<std::pair<Id, double>> sortedDensities(densities.begin(),
                                                         densities.end());

      // Sort by density descending
      std::sort(sortedDensities.begin(),
                sortedDensities.end(),
                [](auto const& a, auto const& b) { return a.second > b.second; });
      std::unordered_set<Id> optimizedNodes;

      for (auto const& [nodeId, density] : sortedDensities) {
        auto const& inNeighbours{this->graph().node(nodeId)->ingoingEdges()};
        for (auto const& inEdgeId : inNeighbours) {
          auto const& pStreet{this->graph().edge(inEdgeId)};
          auto const& sourceId{pStreet->source()};
          if (!densities.contains(sourceId) || optimizedNodes.contains(sourceId)) {
            continue;
          }
          auto const& neighbourDensity{densities.at(sourceId)};
          if (neighbourDensity < threshold * density) {
            continue;
          }
          // Try to green-wave the situation
          auto& tl{dynamic_cast<TrafficLight&>(*this->graph().node(sourceId))};
          tl.increasePhases(pStreet->length() /
                            (pStreet->maxSpeed() * (1. - 0.6 * pStreet->density(true))));
          optimizedNodes.insert(sourceId);
          if (logStream.has_value()) {
            *logStream << std::format("\nNew cycles for {}", tl);
          }
        }
      }
    }
    // Cleaning variables
    for (auto& [streetId, pair] : m_queuesAtTrafficLights) {
      for (auto& [direction, value] : pair) {
        value = 0.;
      }
    }
    m_previousOptimizationTime = this->time_step();
    if (logStream.has_value()) {
      logStream->close();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Size RoadDynamics<delay_t>::nAgents() const {
    return m_nAgents;
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
    if (m_travelDTs.empty()) {
      return Measurement(0., 0.);
    }
    std::vector<double> travelDistances;
    travelDistances.reserve(m_travelDTs.size());
    for (auto const& [distance, time] : m_travelDTs) {
      travelDistances.push_back(distance);
    }
    if (clearData) {
      m_travelDTs.clear();
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
  std::unordered_map<Id, std::unordered_map<Id, double>> const
  RoadDynamics<delay_t>::normalizedTurnCounts() const noexcept {
    std::unordered_map<Id, std::unordered_map<Id, double>> normalizedTurnCounts;
    for (auto const& [fromId, map] : m_turnCounts) {
      auto const sum{
          std::accumulate(map.begin(), map.end(), 0., [](auto const sum, auto const& p) {
            return sum + static_cast<double>(p.second);
          })};
      for (auto const& [toId, count] : map) {
        normalizedTurnCounts[fromId][toId] =
            sum == 0. ? 0. : static_cast<double>(count) / sum;
      }
    }
    return normalizedTurnCounts;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  double RoadDynamics<delay_t>::streetMeanSpeed(Id streetId) const {
    auto const& pStreet{this->graph().edge(streetId)};
    auto const nAgents{pStreet->nAgents()};
    if (nAgents == 0) {
      return 0.;
    }
    double speed{0.};
    for (auto const& pAgent : pStreet->movingAgents()) {
      speed += pAgent->speed();
    }
    return speed / nAgents;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanSpeed() const {
    std::vector<double> speeds;
    speeds.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      speeds.push_back(streetMeanSpeed(streetId));
    }
    return Measurement<double>(speeds);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanSpeed(double threshold,
                                                             bool above) const {
    std::vector<double> speeds;
    speeds.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      if (above && (pStreet->density(true) > threshold)) {
        speeds.push_back(streetMeanSpeed(streetId));
      } else if (!above && (pStreet->density(true) < threshold)) {
        speeds.push_back(streetMeanSpeed(streetId));
      }
    }
    return Measurement<double>(speeds);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanDensity(bool normalized) const {
    if (this->graph().edges().empty()) {
      return Measurement(0., 0.);
    }
    std::vector<double> densities;
    densities.reserve(this->graph().nEdges());
    if (normalized) {
      for (const auto& [streetId, pStreet] : this->graph().edges()) {
        densities.push_back(pStreet->density(true));
      }
    } else {
      double sum{0.};
      for (const auto& [streetId, pStreet] : this->graph().edges()) {
        densities.push_back(pStreet->density(false) * pStreet->length());
        sum += pStreet->length();
      }
      if (sum == 0) {
        return Measurement(0., 0.);
      }
      auto meanDensity{std::accumulate(densities.begin(), densities.end(), 0.) / sum};
      return Measurement(meanDensity, 0.);
    }
    return Measurement<double>(densities);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanFlow() const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      flows.push_back(pStreet->density() * this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanFlow(double threshold,
                                                            bool above) const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      if (above && (pStreet->density(true) > threshold)) {
        flows.push_back(pStreet->density() * this->streetMeanSpeed(streetId));
      } else if (!above && (pStreet->density(true) < threshold)) {
        flows.push_back(pStreet->density() * this->streetMeanSpeed(streetId));
      }
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveStreetDensities(std::string filename,
                                                  bool normalized,
                                                  char const separator) const {
    if (filename.empty()) {
      filename =
          this->m_safeDateTime() + '_' + this->m_safeName() + "_street_densities.csv";
    }
    bool bEmptyFile{false};
    {
      std::ifstream file(filename);
      bEmptyFile = file.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + filename + "\" for writing.");
    }
    if (bEmptyFile) {
      file << "datetime" << separator << "time_step";
      for (auto const& [streetId, pStreet] : this->graph().edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->strDateTime() << separator << this->time_step();
    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      // keep 2 decimal digits;
      file << separator << std::scientific << std::setprecision(2)
           << pStreet->density(normalized);
    }
    file << std::endl;
    file.close();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveCoilCounts(const std::string& filename,
                                             bool reset,
                                             char const separator) {
    bool bEmptyFile{false};
    {
      std::ifstream file(filename);
      bEmptyFile = file.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + filename + "\" for writing.");
    }
    if (bEmptyFile) {
      file << "datetime" << separator << "time_step";
      for (auto const& [streetId, pStreet] : this->graph().edges()) {
        if (pStreet->hasCoil()) {
          file << separator << pStreet->counterName();
        }
      }
      file << std::endl;
    }
    file << this->strDateTime() << separator << this->time_step();
    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      if (pStreet->hasCoil()) {
        file << separator << pStreet->counts();
        if (reset) {
          pStreet->resetCounter();
        }
      }
    }
    file << std::endl;
    file.close();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveTravelData(std::string filename, bool reset) {
    if (filename.empty()) {
      filename = this->m_safeDateTime() + '_' + this->m_safeName() + "_travel_data.csv";
    }
    bool bEmptyFile{false};
    {
      std::ifstream file(filename);
      bEmptyFile = file.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + filename + "\" for writing.");
    }
    if (bEmptyFile) {
      file << "datetime;time_step;distances;times;speeds" << std::endl;
    }

    // Construct strings efficiently with proper formatting
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    std::string strTravelDistances, strTravelTimes, strTravelSpeeds;
    strTravelDistances.reserve(m_travelDTs.size() *
                               10);  // Rough estimate for numeric strings
    strTravelTimes.reserve(m_travelDTs.size() * 10);
    strTravelSpeeds.reserve(m_travelDTs.size() * 10);

    for (auto it = m_travelDTs.cbegin(); it != m_travelDTs.cend(); ++it) {
      oss.str("");  // Clear the stream
      oss << it->first;
      strTravelDistances += oss.str();

      oss.str("");
      oss << it->second;
      strTravelTimes += oss.str();

      oss.str("");
      oss << (it->first / it->second);
      strTravelSpeeds += oss.str();

      if (it != m_travelDTs.cend() - 1) {
        strTravelDistances += ',';
        strTravelTimes += ',';
        strTravelSpeeds += ',';
      }
    }

    // Write all data at once
    file << this->strDateTime() << ';' << this->time_step() << ';' << strTravelDistances
         << ';' << strTravelTimes << ';' << strTravelSpeeds << std::endl;

    file.close();
    if (reset) {
      m_travelDTs.clear();
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveMacroscopicObservables(std::string filename,
                                                         char const separator) {
    if (filename.empty()) {
      filename = this->m_safeDateTime() + '_' + this->m_safeName() +
                 "_macroscopic_observables.csv";
    }
    bool bEmptyFile{false};
    {
      std::ifstream file(filename);
      bEmptyFile = file.peek() == std::ifstream::traits_type::eof();
    }
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + filename + "\" for writing.");
    }
    if (bEmptyFile) {
      constexpr auto strHeader{
          "datetime;time_step;n_ghost_agents;n_agents;mean_speed_kph;std_speed_kph;"
          "mean_density_vpk;std_density_vpk;mean_flow_vph;std_flow_vph;mean_"
          "traveltime_m;std_traveltime_m;mean_traveldistance_km;std_traveldistance_"
          "km;mean_travelspeed_kph;std_travelspeed_kph\n"};
      file << strHeader;
    }
    double mean_speed{0.}, mean_density{0.}, mean_flow{0.}, mean_travel_distance{0.},
        mean_travel_time{0.}, mean_travel_speed{0.};
    double std_speed{0.}, std_density{0.}, std_flow{0.}, std_travel_distance{0.},
        std_travel_time{0.}, std_travel_speed{0.};
    auto const& nEdges{this->graph().nEdges()};
    auto const& nData{m_travelDTs.size()};

    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      auto const& speed{this->streetMeanSpeed(streetId) * 3.6};
      auto const& density{pStreet->density() * 1e3};
      auto const& flow{density * speed};
      mean_speed += speed;
      mean_density += density;
      mean_flow += flow;
      std_speed += speed * speed;
      std_density += density * density;
      std_flow += flow * flow;
    }
    mean_speed /= nEdges;
    mean_density /= nEdges;
    mean_flow /= nEdges;
    std_speed = std::sqrt(std_speed / nEdges - mean_speed * mean_speed);
    std_density = std::sqrt(std_density / nEdges - mean_density * mean_density);
    std_flow = std::sqrt(std_flow / nEdges - mean_flow * mean_flow);

    for (auto const& [distance, time] : m_travelDTs) {
      mean_travel_distance += distance * 1e-3;
      mean_travel_time += time / 60.;
      mean_travel_speed += distance / time * 3.6;
      std_travel_distance += distance * distance * 1e-6;
      std_travel_time += time * time / 3600.;
      std_travel_speed += (distance / time) * (distance / time) * 12.96;
    }
    m_travelDTs.clear();

    mean_travel_distance /= nData;
    mean_travel_time /= nData;
    mean_travel_speed /= nData;
    std_travel_distance = std::sqrt(std_travel_distance / nData -
                                    mean_travel_distance * mean_travel_distance);
    std_travel_time =
        std::sqrt(std_travel_time / nData - mean_travel_time * mean_travel_time);
    std_travel_speed =
        std::sqrt(std_travel_speed / nData - mean_travel_speed * mean_travel_speed);

    file << this->strDateTime() << separator;
    file << this->time_step() << separator;
    file << m_agents.size() << separator;
    file << this->nAgents() << separator;
    file << std::scientific << std::setprecision(2);
    file << mean_speed << separator << std_speed << separator;
    file << mean_density << separator << std_density << separator;
    file << mean_flow << separator << std_flow << separator;
    file << mean_travel_time << separator << std_travel_time << separator;
    file << mean_travel_distance << separator << std_travel_distance << separator;
    file << mean_travel_speed << separator << std_travel_speed << std::endl;

    file.close();
  }
};  // namespace dsf::mobility
