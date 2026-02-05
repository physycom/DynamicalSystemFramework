/// @file       /src/dsf/headers/RoadDynamics.hpp
/// @brief      Defines the RoadDynamics class.
///
/// @details    This file contains the definition of the RoadDynamics class.
///             The RoadDynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include <algorithm>
#include <atomic>
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
#include <iostream>
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

static constexpr auto CACHE_FOLDER = "./.dsfcache/";
static constexpr auto U_TURN_PENALTY_FACTOR = 0.1;

namespace dsf::mobility {
  /// @brief The RoadDynamics class represents the dynamics of the network.
  /// @tparam delay_t The type of the agent's delay
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  class RoadDynamics : public Dynamics<RoadNetwork> {
    std::vector<Id> m_nodeIndices;
    std::vector<std::unique_ptr<Agent>> m_agents;
    std::unordered_map<Id, std::shared_ptr<Itinerary>> m_itineraries;
    std::unordered_map<Id, double> m_originNodes;
    std::unordered_map<Id, double> m_destinationNodes;
    tbb::concurrent_unordered_map<Id, std::size_t> m_originCounts;
    tbb::concurrent_unordered_map<Id, std::size_t> m_destinationCounts;
    std::atomic<std::size_t> m_nAgents{0}, m_nAddedAgents{0}, m_nInsertedAgents{0},
        m_nKilledAgents{0}, m_nArrivedAgents{0};

  protected:
    std::unordered_map<Id, std::unordered_map<Id, size_t>> m_turnCounts;
    std::unordered_map<Id, std::array<long, 4>> m_turnMapping;
    tbb::concurrent_unordered_map<Id, std::unordered_map<Direction, double>>
        m_queuesAtTrafficLights;
    tbb::concurrent_vector<std::pair<double, double>> m_travelDTs;
    std::time_t m_previousOptimizationTime{0};

  private:
    std::function<double(std::unique_ptr<Street> const&)> m_weightFunction;
    std::optional<double> m_errorProbability{std::nullopt};
    std::optional<double> m_passageProbability{std::nullopt};
    std::optional<double> m_meanTravelDistance{std::nullopt};
    std::optional<std::time_t> m_meanTravelTime{std::nullopt};
    double m_weightTreshold;
    std::optional<double> m_timeToleranceFactor{std::nullopt};
    std::optional<delay_t> m_dataUpdatePeriod;
    bool m_bCacheEnabled;
    bool m_forcePriorities{false};
    // Saving variables
    std::time_t m_savingInterval{0};
    bool m_bSaveStreetSpeeds{false};
    bool m_bSaveStreetDensities{false};
    bool m_bSaveMacroscopicObservables{false};

  private:
    /// @brief Kill an agent
    /// @param pAgent A std::unique_ptr to the agent to kill
    std::unique_ptr<Agent> m_killAgent(std::unique_ptr<Agent> pAgent);
    /// @brief Update the path of a single itinerary using Dijsktra's algorithm
    /// @param pItinerary An std::shared_ptr to the itinerary
    void m_updatePath(std::shared_ptr<Itinerary> const& pItinerary);

    /// @brief Get the next street id
    /// @param pAgent A std::unique_ptr to the agent
    /// @param pNode A std::unique_ptr to the current node
    /// @return Id The id of the randomly selected next street
    std::optional<Id> m_nextStreetId(const std::unique_ptr<Agent>& pAgent,
                                     const std::unique_ptr<RoadJunction>& pNode);
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

    virtual double m_speedFactor(double const& density) const = 0;
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
    /// @brief Set the mean distance travelled by a random agent. The distance will be sampled from an exponential distribution with this mean.
    /// @param meanTravelDistance The mean distance
    /// @throw std::invalid_argument If the mean distance is negative
    inline void setMeanTravelDistance(double const meanTravelDistance) {
      meanTravelDistance > 0. ? m_meanTravelDistance = meanTravelDistance
                              : throw std::invalid_argument(
                                    "RoadDynamics::setMeanTravelDistance: "
                                    "meanTravelDistance must be positive");
    };
    /// @brief Set the mean travel time for random agents. The travel time will be sampled from an exponential distribution with this mean.
    /// @param meanTravelTime The mean travel time
    inline void setMeanTravelTime(std::time_t const meanTravelTime) noexcept {
      m_meanTravelTime = meanTravelTime;
    };
    /// @brief Set the origin nodes. If the provided map is empty, the origin nodes are set using the streets' stationary weights.
    /// NOTE: the default stationary weights are 1.0 so, if not set, this is equivalent to setting uniform weights.
    /// @param originNodes The origin nodes
    void setOriginNodes(std::unordered_map<Id, double> const& originNodes = {});
    /// @brief Set the destination nodes
    /// @param destinationNodes The destination nodes
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
    /// @param throw_on_empty If true, throws an exception if an itinerary has an empty path (default is true)
    /// If false, removes the itinerary with empty paths and the associated node from the origin/destination nodes
    /// @throws std::runtime_error if throw_on_empty is true and an itinerary has an empty path
    void updatePaths(bool const throw_on_empty = true);
    /// @brief Add agents uniformly on the road network
    /// @param nAgents The number of agents to add
    /// @param itineraryId The id of the itinerary to use (default is std::nullopt)
    /// @throw std::runtime_error If there are no itineraries
    void addAgentsUniformly(Size nAgents, std::optional<Id> itineraryId = std::nullopt);

    template <typename TContainer>
      requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
               std::is_same_v<TContainer, std::map<Id, double>>)
    void addRandomAgents(std::size_t nAgents, TContainer const& spawnWeights);

    void addRandomAgents(std::size_t nAgents);
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
      requires(std::is_constructible_v<Agent, Id, std::time_t, TArgs...>)
    void addAgent(TArgs&&... args);

    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, Id, std::time_t, TArgs...>)
    void addAgents(std::size_t const nAgents, TArgs&&... args);

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
    void addItinerary(std::shared_ptr<Itinerary> itinerary);

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
    inline auto const& itineraries() const noexcept { return m_itineraries; }
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

    /// @brief Get the origin counts of the agents
    /// @param bReset If true, the origin counts are cleared (default is true)
    tbb::concurrent_unordered_map<Id, std::size_t> originCounts(
        bool const bReset = true) noexcept;
    /// @brief Get the destination counts of the agents
    /// @param bReset If true, the destination counts are cleared (default is true)
    tbb::concurrent_unordered_map<Id, std::size_t> destinationCounts(
        bool const bReset = true) noexcept;

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

    /// @brief Save the street densities to the connected database
    /// @param normalized If true, the densities are normalized in [0, 1] dividing by the street capacity attribute
    /// @throw std::runtime_error if no database is connected
    void saveStreetDensities(bool const normalized = true) const;
    /// @brief Save the street speeds to the connected database
    /// @param bNormalized If true, the speeds are normalized in [0, 1] dividing by the street maxSpeed attribute
    /// @throw std::runtime_error if no database is connected
    void saveStreetSpeeds(bool bNormalized = false) const;
    /// @brief Save the street input counts to the connected database
    /// @param reset If true, the input counts are cleared after the computation
    /// @throw std::runtime_error if no database is connected
    /// @details NOTE: counts are saved only if the street has a coil on it
    void saveCoilCounts(bool reset = false);
    /// @brief Save the travel data of the agents to the connected database
    /// @param reset If true, the travel speeds are cleared after the computation
    /// @throw std::runtime_error if no database is connected
    void saveTravelData(bool reset = false);
    /// @brief Save the main macroscopic observables to the connected database
    /// @throw std::runtime_error if no database is connected
    /// @details The table contains the following columns:
    /// - datetime: the datetime of the simulation
    /// - time_step: the current time step
    /// - n_ghost_agents: the number of agents waiting to be inserted in the simulation
    /// - n_agents: the number of agents currently in the simulation
    /// - mean_speed, std_speed (km/h): the mean speed of the agents
    /// - mean_density, std_density (veh/km): the mean density of the streets
    /// - mean_flow, std_flow (veh/h): the mean flow of the streets
    /// - mean_traveltime, std_traveltime (min): the mean travel time of the agents
    /// - mean_traveldistance, std_traveldistance (km): the mean travel distance of the agents
    /// - mean_travelspeed, std_travelspeed (km/h): the mean travel speed of the agents
    ///
    /// NOTE: the mean density is normalized in [0, 1] and travel data is reset after saving
    void saveMacroscopicObservables();

    /// @brief Print a summary of the dynamics to an output stream
    /// @param os The output stream to write to (default is std::cout)
    /// @details The summary includes:
    /// - The RoadNetwork description (nodes, edges, capacity, intersections, traffic lights, roundabouts, coil sensors)
    /// - Number of inserted agents
    /// - Number of added agents
    /// - Number of arrived agents
    /// - Number of killed agents
    /// - Current number of agents in the simulation
    void summary(std::ostream& os = std::cout) const;
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  RoadDynamics<delay_t>::RoadDynamics(RoadNetwork& graph,
                                      bool useCache,
                                      std::optional<unsigned int> seed,
                                      PathWeight const weightFunction,
                                      std::optional<double> weightTreshold)
      : Dynamics<RoadNetwork>(graph, seed), m_bCacheEnabled{useCache} {
    this->setWeightFunction(weightFunction, weightTreshold);
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(CACHE_FOLDER)) {
        std::filesystem::create_directory(CACHE_FOLDER);
      }
      spdlog::info("Cache enabled (default folder is {})", CACHE_FOLDER);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      m_nodeIndices.push_back(nodeId);
    }
    for (auto const& [nodeId, weight] : this->m_destinationNodes) {
      m_itineraries.emplace(nodeId, std::make_shared<Itinerary>(nodeId, nodeId));
    }
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
  std::unique_ptr<Agent> RoadDynamics<delay_t>::m_killAgent(
      std::unique_ptr<Agent> pAgent) {
    spdlog::trace("Killing agent {}", *pAgent);
    m_travelDTs.push_back({pAgent->distance(),
                           static_cast<double>(this->time_step() - pAgent->spawnTime())});
    --m_nAgents;
    auto const& streetId = pAgent->streetId();
    if (streetId.has_value()) {
      auto const& pStreet{this->graph().edge(streetId.value())};
      auto const& pNode{this->graph().node(pStreet->target())};
      auto [it, bInserted] = m_destinationCounts.insert({pNode->id(), 1});
      if (!bInserted) {
        ++it->second;
      }
    }
    return pAgent;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_updatePath(std::shared_ptr<Itinerary> const& pItinerary) {
    if (m_bCacheEnabled) {
      auto const& file = std::format("{}{}.ity", CACHE_FOLDER, pItinerary->id());
      if (std::filesystem::exists(file)) {
        pItinerary->load(file);
        spdlog::debug("Loaded cached path for itinerary {}", pItinerary->id());
        return;
      }
    }
    auto const oldSize{pItinerary->path().size()};

    auto const& path{this->graph().allPathsTo(
        pItinerary->destination(), m_weightFunction, m_weightTreshold)};
    pItinerary->setPath(path);
    auto const newSize{pItinerary->path().size()};
    if (oldSize > 0 && newSize != oldSize) {
      spdlog::debug("Path for itinerary {} changed size from {} to {}",
                    pItinerary->id(),
                    oldSize,
                    newSize);
    }
    if (m_bCacheEnabled) {
      pItinerary->save(std::format("{}{}.ity", CACHE_FOLDER, pItinerary->id()));
      spdlog::debug("Saved path in cache for itinerary {}", pItinerary->id());
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  std::optional<Id> RoadDynamics<delay_t>::m_nextStreetId(
      const std::unique_ptr<Agent>& pAgent, const std::unique_ptr<RoadJunction>& pNode) {
    spdlog::trace("Computing m_nextStreetId for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Get current street information
    std::optional<Id> previousNodeId = std::nullopt;
    std::set<Id> forbiddenTurns;
    double speedCurrent = 1.0;
    double stationaryWeightCurrent = 1.0;
    if (pAgent->streetId().has_value()) {
      auto const& pStreetCurrent{this->graph().edge(pAgent->streetId().value())};
      previousNodeId = pStreetCurrent->source();
      forbiddenTurns = pStreetCurrent->forbiddenTurns();
      speedCurrent = pStreetCurrent->maxSpeed();
      stationaryWeightCurrent = pStreetCurrent->stationaryWeight();
    }

    // Get path targets for non-random agents
    std::vector<Id> pathTargets;
    if (!pAgent->isRandom()) {
      pathTargets = pAgent->itinerary()->path().at(pNode->id());
    }

    // Calculate transition probabilities for all valid outgoing edges
    std::unordered_map<Id, double> transitionProbabilities;
    double cumulativeProbability = 0.0;

    for (const auto outEdgeId : outgoingEdges) {
      auto const& pStreetOut{this->graph().edge(outEdgeId)};

      // Check if this is a valid path target for non-random agents
      bool bIsPathTarget = false;
      if (!pathTargets.empty()) {
        bIsPathTarget =
            std::find(pathTargets.cbegin(), pathTargets.cend(), pStreetOut->target()) !=
            pathTargets.cend();
      }

      if (forbiddenTurns.contains(outEdgeId) && !bIsPathTarget) {
        continue;
      }

      if (!pathTargets.empty()) {
        if (!this->m_errorProbability.has_value() && !bIsPathTarget) {
          continue;
        }
      }

      // Calculate base probability
      auto const speedNext{pStreetOut->maxSpeed()};
      double const stationaryWeightNext = pStreetOut->stationaryWeight();
      auto const weightRatio{stationaryWeightNext /
                             stationaryWeightCurrent};  // SQRT (p_i / p_j)
      double probability = speedCurrent * speedNext * std::sqrt(weightRatio);

      // Apply error probability for non-random agents
      if (this->m_errorProbability.has_value() && !pathTargets.empty()) {
        probability *=
            (bIsPathTarget
                 ? (1. - this->m_errorProbability.value())
                 : this->m_errorProbability.value() /
                       static_cast<double>(outgoingEdges.size() - pathTargets.size()));
      }

      // Handle U-turns
      if (previousNodeId.has_value() && pStreetOut->target() == previousNodeId.value()) {
        if (pNode->isRoundabout()) {
          probability *= U_TURN_PENALTY_FACTOR;
        } else if (!bIsPathTarget) {
          continue;  // No U-turns allowed
        }
      }

      transitionProbabilities[pStreetOut->id()] = probability;
      cumulativeProbability += probability;
    }

    // Select street based on weighted probabilities
    if (transitionProbabilities.empty()) {
      spdlog::debug("No valid transitions found for {} at {}", *pAgent, *pNode);
      return std::nullopt;
    }
    if (transitionProbabilities.size() == 1) {
      auto const& onlyStreetId = transitionProbabilities.cbegin()->first;
      spdlog::debug("Only one valid transition for {} at {}: street {}",
                    *pAgent,
                    *pNode,
                    onlyStreetId);
      return onlyStreetId;
    }

    std::uniform_real_distribution<double> uniformDist{0., cumulativeProbability};
    auto const randValue = uniformDist(this->m_generator);
    Id fallbackStreetId;
    double accumulated = 0.0;
    for (const auto& [targetStreetId, probability] : transitionProbabilities) {
      accumulated += probability;
      fallbackStreetId = targetStreetId;
      if (randValue < accumulated) {
        return targetStreetId;
      }
    }
    // Return last one as fallback
    spdlog::debug(
        "Fallback selection for {} at {}: street {}", *pAgent, *pNode, fallbackStreetId);
    return fallbackStreetId;
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveStreet(const std::unique_ptr<Street>& pStreet,
                                             bool reinsert_agents) {
    auto const nLanes = pStreet->nLanes();
    // Enqueue moving agents if their free time is up
    while (!pStreet->movingAgents().empty()) {
      auto const& pAgent{pStreet->movingAgents().top()};
      if (pAgent->freeTime() < this->time_step()) {
        break;
      }
      pAgent->setSpeed(0.);
      bool bArrived{false};
      if (!pAgent->isRandom()) {
        if (pAgent->itinerary()->destination() == pStreet->target()) {
          pAgent->updateItinerary();
        }
        if (pAgent->itinerary()->destination() == pStreet->target()) {
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
          this->m_nextStreetId(pAgent, this->graph().node(pStreet->target()));
      if (!nextStreetId.has_value()) {
        spdlog::debug(
            "No next street found for agent {} at node {}", *pAgent, pStreet->target());
        if (pAgent->isRandom()) {
          std::uniform_int_distribution<size_t> laneDist{0,
                                                         static_cast<size_t>(nLanes - 1)};
          pStreet->enqueue(laneDist(this->m_generator));
          continue;
        }
        throw std::runtime_error(std::format(
            "No next street found for agent {} at node {}", *pAgent, pStreet->target()));
      }
      auto const& pNextStreet{this->graph().edge(nextStreetId.value())};
      pAgent->setNextStreetId(pNextStreet->id());
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
      if (i == std::ceil(transportCapacity) - 1) {
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          spdlog::trace("Skipping due to fractional capacity {:.2f} < random value",
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
          spdlog::trace("Skipping due to time {} < free time {}",
                        this->time_step(),
                        pAgentTemp->freeTime());
          continue;
        }

        if (m_timeToleranceFactor.has_value()) {
          auto const timeDiff{this->time_step() - pAgentTemp->freeTime()};
          auto const timeTolerance{m_timeToleranceFactor.value() *
                                   std::ceil(pStreet->length() / pStreet->maxSpeed())};
          if (timeDiff > timeTolerance) {
            spdlog::debug(
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
            this->m_killAgent(pStreet->dequeue(queueIndex, this->time_step()));
            ++m_nKilledAgents;
            continue;
          }
        }
        pAgentTemp->setSpeed(0.);
        const auto& destinationNode{this->graph().node(pStreet->target())};
        if (destinationNode->isFull()) {
          spdlog::trace("Skipping due to full destination node {}", *destinationNode);
          continue;
        }
        if (destinationNode->isTrafficLight()) {
          auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
          auto const direction{pStreet->laneMapping().at(queueIndex)};
          if (!tl.isGreen(pStreet->id(), direction)) {
            spdlog::trace("Skipping due to red light on street {} and direction {}",
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
          if (destinationNode->id() == pAgentTemp->itinerary()->destination()) {
            bArrived = true;
            spdlog::debug("Agent {} has arrived at destination node {}",
                          pAgentTemp->id(),
                          destinationNode->id());
          }
        } else {
          if (!pAgentTemp->nextStreetId().has_value()) {
            bArrived = true;
            spdlog::debug("Random agent {} has arrived at destination node {}",
                          pAgentTemp->id(),
                          destinationNode->id());
          } else if (pAgentTemp->hasArrived(this->time_step())) {
            bArrived = true;
          }
        }
        if (bArrived) {
          auto pAgent =
              this->m_killAgent(pStreet->dequeue(queueIndex, this->time_step()));
          ++m_nArrivedAgents;
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
        auto pAgent{pStreet->dequeue(queueIndex, this->time_step())};
        spdlog::debug(
            "{} at time {} has been dequeued from street {} and enqueued on street {} "
            "with free time {}.",
            *pAgent,
            this->time_step(),
            pStreet->id(),
            nextStreet->id(),
            pAgent->freeTime());
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
          nextStreet->addAgent(std::move(pAgent), this->time_step());
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
          nextStreet->addAgent(std::move(pAgent), this->time_step());
        } else {
          return;
        }
      }
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveAgents() {
    if (m_agents.empty()) {
      spdlog::trace("No agents to process.");
      return;
    }
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
        auto const nextStreetId{this->m_nextStreetId(pAgent, pSourceNode)};
        if (!nextStreetId.has_value()) {
          spdlog::debug(
              "No next street found for agent {} at node {}", *pAgent, pSourceNode->id());
          itAgent = m_agents.erase(itAgent);
          continue;
        }
        pAgent->setNextStreetId(nextStreetId.value());
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
    if (originNodes.empty()) {
      // If no origin nodes are provided, try to set origin nodes basing on streets' stationary weights
      double totalStationaryWeight = 0.0;
      for (auto const& [edgeId, pEdge] : this->graph().edges()) {
        auto const& weight = pEdge->stationaryWeight();
        m_originNodes[pEdge->source()] += weight;
        totalStationaryWeight += weight;
      }
      for (auto& [nodeId, weight] : m_originNodes) {
        weight /= totalStationaryWeight;
      }
      return;
    }
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
    m_itineraries.clear();
    m_destinationNodes.clear();
    m_destinationNodes.reserve(destinationNodes.size());
    auto sumWeights{0.};
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &sumWeights](auto const& pair) -> void {
                    sumWeights += pair.second;
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
    m_itineraries.clear();
    auto const numNodes{destinationNodes.size()};
    m_destinationNodes.clear();
    m_destinationNodes.reserve(numNodes);
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &numNodes](auto const& nodeId) -> void {
                    this->m_destinationNodes[nodeId] = 1. / numNodes;
                    this->addItinerary(nodeId, nodeId);
                  });
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_convertible_v<typename TContainer::value_type, Id>)
  void RoadDynamics<delay_t>::setDestinationNodes(TContainer const& destinationNodes) {
    m_itineraries.clear();
    auto const numNodes{destinationNodes.size()};
    m_destinationNodes.clear();
    m_destinationNodes.reserve(numNodes);
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &numNodes](auto const& nodeId) -> void {
                    this->m_destinationNodes[nodeId] = 1. / numNodes;
                    this->addItinerary(nodeId, nodeId);
                  });
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::updatePaths(bool const throw_on_empty) {
    spdlog::debug("Init updating paths...");
    tbb::concurrent_vector<Id> emptyItineraries;
    tbb::parallel_for_each(
        this->itineraries().cbegin(),
        this->itineraries().cend(),
        [this, throw_on_empty, &emptyItineraries](auto const& pair) -> void {
          auto const& pItinerary{pair.second};
          this->m_updatePath(pItinerary);
          if (pItinerary->empty()) {
            if (!throw_on_empty) {
              spdlog::warn("No path found for itinerary {} with destination node {}",
                           pItinerary->id(),
                           pItinerary->destination());
              emptyItineraries.push_back(pItinerary->id());
              return;
            }
            throw std::runtime_error(
                std::format("No path found for itinerary {} with destination node {}",
                            pItinerary->id(),
                            pItinerary->destination()));
          }
        });
    if (!emptyItineraries.empty()) {
      spdlog::warn("Removing {} itineraries with no valid path from the dynamics.",
                   emptyItineraries.size());
      for (auto const& id : emptyItineraries) {
        auto const destination = m_itineraries.at(id)->destination();
        m_destinationNodes.erase(destination);
        m_originNodes.erase(destination);
        m_itineraries.erase(id);
      }
    }
    spdlog::debug("End updating paths.");
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgentsUniformly(Size nAgents,
                                                 std::optional<Id> optItineraryId) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
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
    std::shared_ptr<Itinerary> pItinerary;
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
        pItinerary = itineraryIt->second;
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
      this->addAgent(pItinerary, street->source());
      auto& pAgent{this->m_agents.back()};
      pAgent->setStreetId(street->id());
      this->setAgentSpeed(pAgent);
      pAgent->setFreeTime(this->time_step() +
                          std::ceil(street->length() / pAgent->speed()));
      street->addAgent(std::move(pAgent), this->time_step());
      this->m_agents.pop_back();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
             std::is_same_v<TContainer, std::map<Id, double>>)
  void RoadDynamics<delay_t>::addRandomAgents(std::size_t nAgents,
                                              TContainer const& spawnWeights) {
    m_nAddedAgents += nAgents;
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    std::exponential_distribution<double> distDist{1. /
                                                   m_meanTravelDistance.value_or(1.)};
    std::exponential_distribution<double> timeDist{1. / m_meanTravelTime.value_or(1.)};
    auto const bUniformSpawn{spawnWeights.empty()};
    auto const bSingleSource{spawnWeights.size() == 1};
    while (nAgents--) {
      if (bUniformSpawn) {
        this->addAgent();
      } else if (bSingleSource) {
        this->addAgent(nullptr, spawnWeights.begin()->first);
      } else {
        auto const randValue{uniformDist(this->m_generator)};
        double cumulativeWeight{0.};
        for (auto const& [spawnNodeId, weight] : spawnWeights) {
          cumulativeWeight += weight;
          if (randValue <= cumulativeWeight) {
            this->addAgent(nullptr, spawnNodeId);
            break;
          }
        }
      }
      if (m_meanTravelDistance.has_value()) {
        auto const& pAgent{this->m_agents.back()};
        pAgent->setMaxDistance(distDist(this->m_generator));
      }
      if (m_meanTravelTime.has_value()) {
        auto const& pAgent{this->m_agents.back()};
        pAgent->setMaxTime(timeDist(this->m_generator));
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addRandomAgents(std::size_t nAgents) {
    addRandomAgents(nAgents, this->m_originNodes);
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
             std::is_same_v<TContainer, std::map<Id, double>>)
  void RoadDynamics<delay_t>::addAgentsRandomly(Size nAgents,
                                                const TContainer& src_weights,
                                                const TContainer& dst_weights) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
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
        spdlog::debug("Destination {} not reachable from source {}. Skipping agent.",
                      *dstId,
                      *srcId);
        --nAgents;
        continue;
      }

      this->addAgent(itineraryIt->second, *srcId);
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
    ++m_nInsertedAgents;
    spdlog::trace("Added {}", *m_agents.back());
    auto const& optNodeId{m_agents.back()->srcNodeId()};
    if (optNodeId.has_value()) {
      auto [it, bInserted] = m_originCounts.insert({*optNodeId, 1});
      if (!bInserted) {
        ++it->second;
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, Id, std::time_t, TArgs...>)
  void RoadDynamics<delay_t>::addAgent(TArgs&&... args) {
    addAgent(std::make_unique<Agent>(
        this->m_nInsertedAgents, this->time_step(), std::forward<TArgs>(args)...));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, Id, std::time_t, TArgs...>)
  void RoadDynamics<delay_t>::addAgents(std::size_t const nAgents, TArgs&&... args) {
    for (size_t i{0}; i < nAgents; ++i) {
      addAgent(std::make_unique<Agent>(
          this->m_nInsertedAgents, this->time_step(), std::forward<TArgs>(args)...));
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Itinerary, TArgs...>)
  void RoadDynamics<delay_t>::addItinerary(TArgs&&... args) {
    addItinerary(std::make_shared<Itinerary>(std::forward<TArgs>(args)...));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addItinerary(std::shared_ptr<Itinerary> itinerary) {
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
  tbb::concurrent_unordered_map<Id, std::size_t> RoadDynamics<delay_t>::originCounts(
      bool const bReset) noexcept {
    if (!bReset) {
      return m_originCounts;
    }
    auto const tempCounts{std::move(m_originCounts)};
    m_originCounts.clear();
    return tempCounts;
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  tbb::concurrent_unordered_map<Id, std::size_t> RoadDynamics<delay_t>::destinationCounts(
      bool const bReset) noexcept {
    if (!bReset) {
      return m_destinationCounts;
    }
    auto const tempCounts{std::move(m_destinationCounts)};
    m_destinationCounts.clear();
    return tempCounts;
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
      auto const speedMeasure = pStreet->meanSpeed();
      if (speedMeasure.is_valid) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      }
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
      auto const speedMeasure = pStreet->meanSpeed();
      if (!speedMeasure.is_valid) {
        continue;
      }
      if (above && (pStreet->density(true) > threshold)) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      } else if (!above && (pStreet->density(true) < threshold)) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveStreetDensities(bool const normalized) const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS street_densities ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "street_id INTEGER NOT NULL, "
        "density REAL NOT NULL)");

    // Begin transaction for better performance
    SQLite::Transaction transaction(*this->database());
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO street_densities (datetime, time_step, street_id, density) "
        "VALUES (?, ?, ?, ?)");

    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      insertStmt.bind(1, this->strDateTime());
      insertStmt.bind(2, static_cast<int64_t>(this->time_step()));
      insertStmt.bind(3, static_cast<int64_t>(streetId));
      insertStmt.bind(4, pStreet->density(normalized));
      insertStmt.exec();
      insertStmt.reset();
    }
    transaction.commit();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveStreetSpeeds(bool const bNormalized) const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS street_speeds ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "street_id INTEGER NOT NULL, "
        "speed REAL, "
        "std REAL)");

    // Begin transaction for better performance
    SQLite::Transaction transaction(*this->database());
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO street_speeds (datetime, time_step, street_id, speed, std) "
        "VALUES (?, ?, ?, ?, ?)");

    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      auto const measure = pStreet->meanSpeed(true);
      insertStmt.bind(1, this->strDateTime());
      insertStmt.bind(2, static_cast<int64_t>(this->time_step()));
      insertStmt.bind(3, static_cast<int64_t>(streetId));

      if (!measure.is_valid) {
        // NULL for invalid speeds
        insertStmt.bind(4);
        insertStmt.bind(5);
      } else {
        double speed{measure.mean};
        double std{measure.std};
        if (bNormalized) {
          speed /= pStreet->maxSpeed();
          std /= pStreet->maxSpeed();
        }
        insertStmt.bind(4, speed);
        insertStmt.bind(5, std);
      }
      insertStmt.exec();
      insertStmt.reset();
    }
    transaction.commit();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveCoilCounts(bool reset) {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS coil_counts ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "coil_name TEXT NOT NULL, "
        "count INTEGER NOT NULL)");

    // Begin transaction for better performance
    SQLite::Transaction transaction(*this->database());
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO coil_counts (datetime, time_step, coil_name, count) "
        "VALUES (?, ?, ?, ?)");

    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      if (pStreet->hasCoil()) {
        insertStmt.bind(1, this->strDateTime());
        insertStmt.bind(2, static_cast<int64_t>(this->time_step()));
        insertStmt.bind(3, pStreet->counterName());
        insertStmt.bind(4, static_cast<int64_t>(pStreet->counts()));
        insertStmt.exec();
        insertStmt.reset();
        if (reset) {
          pStreet->resetCounter();
        }
      }
    }
    transaction.commit();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveTravelData(bool reset) {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS travel_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "distance REAL NOT NULL, "
        "time REAL NOT NULL, "
        "speed REAL NOT NULL)");

    // Begin transaction for better performance
    SQLite::Transaction transaction(*this->database());
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO travel_data (datetime, time_step, distance, time, speed) "
        "VALUES (?, ?, ?, ?, ?)");

    for (auto const& [distance, time] : m_travelDTs) {
      insertStmt.bind(1, this->strDateTime());
      insertStmt.bind(2, static_cast<int64_t>(this->time_step()));
      insertStmt.bind(3, distance);
      insertStmt.bind(4, time);
      insertStmt.bind(5, distance / time);
      insertStmt.exec();
      insertStmt.reset();
    }
    transaction.commit();

    if (reset) {
      m_travelDTs.clear();
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveMacroscopicObservables() {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS macroscopic_observables ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "n_ghost_agents INTEGER NOT NULL, "
        "n_agents INTEGER NOT NULL, "
        "mean_speed_kph REAL NOT NULL, "
        "std_speed_kph REAL NOT NULL, "
        "mean_density_vpk REAL NOT NULL, "
        "std_density_vpk REAL NOT NULL, "
        "mean_flow_vph REAL NOT NULL, "
        "std_flow_vph REAL NOT NULL, "
        "mean_traveltime_m REAL NOT NULL, "
        "std_traveltime_m REAL NOT NULL, "
        "mean_traveldistance_km REAL NOT NULL, "
        "std_traveldistance_km REAL NOT NULL, "
        "mean_travelspeed_kph REAL NOT NULL, "
        "std_travelspeed_kph REAL NOT NULL)");

    double mean_speed{0.}, mean_density{0.}, mean_flow{0.}, mean_travel_distance{0.},
        mean_travel_time{0.}, mean_travel_speed{0.};
    double std_speed{0.}, std_density{0.}, std_flow{0.}, std_travel_distance{0.},
        std_travel_time{0.}, std_travel_speed{0.};
    auto const& nEdges{this->graph().nEdges()};
    auto const& nData{m_travelDTs.size()};
    std::size_t nValidSpeeds{0};

    for (auto const& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed();
      auto const& density{pStreet->density() * 1e3};
      if (speedMeasure.is_valid) {
        auto const speed = speedMeasure.mean * 3.6;  // to kph
        auto const speed_std = speedMeasure.std * 3.6;
        mean_speed += speed;
        std_speed += speed * speed + speed_std * speed_std;

        auto const& flow{density * speed};
        mean_flow += flow;
        std_flow += speed_std;

        ++nValidSpeeds;
      }
      mean_density += density;
      std_density += density * density;
    }
    mean_speed /= nValidSpeeds;
    mean_density /= nEdges;
    mean_flow /= nValidSpeeds;
    std_speed = std::sqrt(std_speed / nValidSpeeds - mean_speed * mean_speed);
    std_density = std::sqrt(std_density / nEdges - mean_density * mean_density);
    std_flow = std::sqrt(std_flow / nValidSpeeds - mean_flow * mean_flow);

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

    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO macroscopic_observables ("
        "datetime, time_step, n_ghost_agents, n_agents, "
        "mean_speed_kph, std_speed_kph, mean_density_vpk, std_density_vpk, "
        "mean_flow_vph, std_flow_vph, mean_traveltime_m, std_traveltime_m, "
        "mean_traveldistance_km, std_traveldistance_km, mean_travelspeed_kph, "
        "std_travelspeed_kph) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

    insertStmt.bind(1, this->strDateTime());
    insertStmt.bind(2, static_cast<int64_t>(this->time_step()));
    insertStmt.bind(3, static_cast<int64_t>(m_agents.size()));
    insertStmt.bind(4, static_cast<int64_t>(this->nAgents()));
    insertStmt.bind(5, mean_speed);
    insertStmt.bind(6, std_speed);
    insertStmt.bind(7, mean_density);
    insertStmt.bind(8, std_density);
    insertStmt.bind(9, mean_flow);
    insertStmt.bind(10, std_flow);
    insertStmt.bind(11, mean_travel_time);
    insertStmt.bind(12, std_travel_time);
    insertStmt.bind(13, mean_travel_distance);
    insertStmt.bind(14, std_travel_distance);
    insertStmt.bind(15, mean_travel_speed);
    insertStmt.bind(16, std_travel_speed);
    insertStmt.exec();
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::summary(std::ostream& os) const {
    os << "RoadDynamics Summary:\n";
    this->graph().describe(os);
    os << "\nNumber of added agents: " << m_nAddedAgents << '\n'
       << "Number of inserted agents: " << m_nInsertedAgents << '\n'
       << "Number of arrived agents: " << m_nArrivedAgents << '\n'
       << "Number of killed agents: " << m_nKilledAgents << '\n'
       << "Current number of agents: " << this->nAgents() << '\n';
  }
}  // namespace dsf::mobility
