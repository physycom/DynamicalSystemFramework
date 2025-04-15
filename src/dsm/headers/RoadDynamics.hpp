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
#include <variant>

#include <tbb/tbb.h>

#include "Dynamics.hpp"
#include "Agent.hpp"
#include "DijkstraWeights.hpp"
#include "Itinerary.hpp"
#include "RoadNetwork.hpp"
#include "SparseMatrix.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

static auto constexpr g_cacheFolder = "./.dsmcache/";

namespace dsm {
  /// @brief The RoadDynamics class represents the dynamics of the network.
  /// @tparam delay_t The type of the agent's delay
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  class RoadDynamics : public Dynamics<RoadNetwork> {
    std::vector<std::unique_ptr<Agent>> m_agents;
    std::unordered_map<Id, std::unique_ptr<Itinerary>> m_itineraries;

  protected:
    std::unordered_map<Id, std::array<unsigned long long, 4>> m_turnCounts;
    std::unordered_map<Id, std::array<long, 4>> m_turnMapping;
    std::unordered_map<Id, std::unordered_map<Direction, double>> m_queuesAtTrafficLights;
    tbb::concurrent_vector<std::pair<double, double>> m_travelDTs;
    Time m_previousOptimizationTime, m_previousSpireTime;

  private:
    std::function<double(const RoadNetwork*, Id, Id)> m_weightFunction;
    std::optional<double> m_errorProbability;
    std::optional<double> m_passageProbability;
    double m_weightTreshold;
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
    /// @brief Increase the turn counts
    virtual void m_increaseTurnCounts(Id streetId, double delta);
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
    void m_evolveAgent(std::unique_ptr<Agent> const& pAgent);

    void m_trafficlightSingleTailOptimizer(double const& beta,
                                           std::optional<std::ofstream>& logStream);

  public:
    /// @brief Construct a new RoadDynamics object
    /// @param graph The graph representing the network
    /// @param useCache If true, the cache is used (default is false)
    /// @param seed The seed for the random number generator (default is std::nullopt)
    /// @param weightFunction The weight function for the Dijkstra's algorithm (default is weight_functions::streetTime)
    /// @param weightTreshold The weight treshold for updating the paths (default is 60.)
    RoadDynamics(RoadNetwork& graph,
                 bool useCache = false,
                 std::optional<unsigned int> seed = std::nullopt,
                 std::function<double(const RoadNetwork*, Id, Id)> weightFunction =
                     weight_functions::streetTime,
                 double weightTreshold = 60.);  // 60 seconds thresholds for paths

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
    /// @brief Set the destination nodes
    /// @param destinationNodes The destination nodes (as an initializer list)
    /// @param updatePaths If true, the paths are updated
    void setDestinationNodes(std::initializer_list<Id> destinationNodes,
                             bool updatePaths = true);
    /// @brief Set the destination nodes
    /// @param destinationNodes A container of destination nodes ids
    /// @param updatePaths If true, the paths are updated
    /// @details The container must have a value_type convertible to Id and begin() and end() methods
    template <typename TContainer>
      requires(std::is_convertible_v<typename TContainer::value_type, Id>)
    void setDestinationNodes(TContainer const& destinationNodes, bool updatePaths = true);

    virtual void setAgentSpeed(std::unique_ptr<Agent> const& pAgent) = 0;

    /// @brief Update the paths of the itineraries based on the given weight function
    void updatePaths();
    /// @brief Add a set of agents to the simulation
    /// @param nAgents The number of agents to add
    /// @param uniformly If true, the agents are added uniformly on the streets
    /// @throw std::runtime_error If there are no itineraries
    void addAgentsUniformly(Size nAgents, std::optional<Id> itineraryId = std::nullopt);
    /// @brief Add a set of agents to the simulation
    /// @param nAgents The number of agents to add
    /// @param src_weights The weights of the source nodes
    /// @param dst_weights The weights of the destination nodes
    /// @param minNodeDistance The minimum distance between the source and destination nodes
    /// @throw std::invalid_argument If the source and destination nodes are the same
    template <typename TContainer>
      requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
               std::is_same_v<TContainer, std::map<Id, double>>)
    void addAgentsRandomly(Size nAgents,
                           const TContainer& src_weights,
                           const TContainer& dst_weights,
                           const std::variant<std::monostate, size_t, double>
                               minNodeDistance = std::monostate{});

    void addAgentsRandomly(Size nAgents,
                           const std::variant<std::monostate, size_t, double>
                               minNodeDistance = std::monostate{});

    /// @brief Add an agent to the simulation
    /// @param agent std::unique_ptr to the agent
    void addAgent(std::unique_ptr<Agent> agent);

    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, Time, TArgs...>)
    void addAgent(TArgs&&... args);

    template <typename... TArgs>
      requires(std::is_constructible_v<Agent, Time, TArgs...>)
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
    /// @param threshold double, The minimum difference between green and red queues to trigger the optimization (n agents - default is 0)
    /// @param optimizationType TrafficLightOptimization, The type of optimization. Default is DOUBLE_TAIL
    /// @param logFile The file into which write the logs (default is empty, meaning no logging)
    /// @details The function cycles over the traffic lights and, if the difference between the two tails is greater than
    ///   the threshold multiplied by the mean capacity of the streets, it changes the green and red times of the traffic light, keeping the total cycle time constant.
    ///   The optimizationType parameter can be set to SINGLE_TAIL to use an algorith which looks only at the incoming street tails or to DOUBLE_TAIL to consider both incoming and outgoing street tails.
    void optimizeTrafficLights(
        double const threshold = 0.,
        TrafficLightOptimization optimizationType = TrafficLightOptimization::DOUBLE_TAIL,
        const std::string& logFile = std::string());

    /// @brief Get the itineraries
    /// @return const std::unordered_map<Id, Itinerary>&, The itineraries
    const std::unordered_map<Id, std::unique_ptr<Itinerary>>& itineraries() const {
      return m_itineraries;
    }
    /// @brief Get the agents
    /// @return const std::unordered_map<Id, Agent<Id>>&, The agents
    const std::vector<std::unique_ptr<Agent>>& agents() const { return m_agents; }
    /// @brief Get the number of agents currently in the simulation
    /// @return Size The number of agents
    size_t nAgents() const;

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

    /// @brief Get the mean speed of the agents in \f$m/s\f$
    /// @return Measurement<double> The mean speed of the agents and the standard deviation
    Measurement<double> agentMeanSpeed() const;
    // TODO: implement the following functions
    // We can implement the base version of these functions by cycling over agents... I won't do it for now.
    // Grufoony - 19/02/2024
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
    /// @brief Get the mean spire input flow of the streets in \f$s^{-1}\f$
    /// @param resetValue If true, the spire input/output flows are cleared after the computation
    /// @return Measurement<double> The mean spire input flow of the streets and the standard deviation
    /// @details The spire input flow is computed as the sum of counts over the product of the number of spires and the time delta
    Measurement<double> meanSpireInputFlow(bool resetValue = true);
    /// @brief Get the mean spire output flow of the streets in \f$s^{-1}\f$
    /// @param resetValue If true, the spire output/input flows are cleared after the computation
    /// @return Measurement<double> The mean spire output flow of the streets and the standard deviation
    /// @details The spire output flow is computed as the sum of counts over the product of the number of spires and the time delta
    Measurement<double> meanSpireOutputFlow(bool resetValue = true);

    /// @brief Save the street densities in csv format
    /// @param filename The name of the file
    /// @param normalized If true, the densities are normalized in [0, 1]
    void saveStreetDensities(const std::string& filename,
                             bool normalized = true,
                             char const separator = ';') const;
    /// @brief Save the street input counts in csv format
    /// @param filename The name of the file
    /// @param reset If true, the input counts are cleared after the computation
    /// @details NOTE: counts are printed only if the street is a spire
    void saveInputStreetCounts(const std::string& filename,
                               bool reset = false,
                               char const separator = ';');
    /// @brief Save the street output counts in csv format
    /// @param filename The name of the file
    /// @param reset If true, the output counts are cleared after the computation
    /// @details NOTE: counts are printed only if the street is a spire
    void saveOutputStreetCounts(const std::string& filename,
                                bool reset = false,
                                char const separator = ';');
    /// @brief Save the travel speeds of the agents in csv format
    /// @param filename The name of the file
    /// @param reset If true, the travel speeds are cleared after the computation
    void saveTravelSpeeds(const std::string& filename, bool reset = false);
    /// @brief Save the main macroscopic observables in csv format
    /// @param filename The name of the file
    /// @param separator The separator character (default is ';')
    /// @details The file contains the following columns:
    /// - time: the time of the simulation
    /// - n_agents: the number of agents currently in the simulation
    /// - mean_speed - mean_speed_std: the mean speed of the agents
    /// - mean_density - mean_density_std: the (normalized) mean density of the streets
    /// - mean_flow - mean_flow_std: the mean flow of the streets
    /// - mean_flow_spires - mean_flow_spires_std: the mean flow of the spires
    /// - mean_traveltime - mean_traveltime_std: the mean travel time of the agents
    /// - mean_traveldistance - mean_traveldistance_err: the mean travel distance of the agents
    /// - mean_travelspeed - mean_travelspeed_std: the mean travel speed of the agents
    ///
    /// NOTE: the mean density is normalized in [0, 1] and reset is true for all observables which have such parameter
    void saveMacroscopicObservables(const std::string& filename,
                                    char const separator = ';');
  };

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  RoadDynamics<delay_t>::RoadDynamics(
      RoadNetwork& graph,
      bool useCache,
      std::optional<unsigned int> seed,
      std::function<double(const RoadNetwork*, Id, Id)> weightFunction,
      double weightTreshold)
      : Dynamics<RoadNetwork>(graph, seed),
        m_previousOptimizationTime{0},
        m_previousSpireTime{0},
        m_weightFunction{weightFunction},
        m_errorProbability{std::nullopt},
        m_passageProbability{std::nullopt},
        m_weightTreshold{weightTreshold},
        m_bCacheEnabled{useCache},
        m_forcePriorities{false} {
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(g_cacheFolder)) {
        std::filesystem::create_directory(g_cacheFolder);
      }
      Logger::info(std::format("Cache enabled (default folder is {})", g_cacheFolder));
    }
    for (const auto& nodeId : this->graph().outputNodes()) {
      this->addItinerary(nodeId, nodeId);
    }
    updatePaths();
    std::for_each(
        this->graph().edges().cbegin(),
        this->graph().edges().cend(),
        [this](auto const& pair) {
          m_turnCounts.emplace(pair.first, std::array<unsigned long long, 4>{0, 0, 0, 0});
          // fill turn mapping as [pair.first, [left street Id, straight street Id, right street Id, U self street Id]]
          m_turnMapping.emplace(pair.first, std::array<long, 4>{-1, -1, -1, -1});
          // Turn mappings
          const auto& srcNodeId = pair.second->target();
          for (const auto& targetId : this->graph().adjacencyMatrix().getRow(srcNodeId)) {
            auto const previousStreetId = srcNodeId * this->graph().nNodes() + targetId;
            auto const& delta{
                pair.second->deltaAngle(this->graph().edge(previousStreetId)->angle())};
            if (std::abs(delta) < std::numbers::pi) {
              if (delta < 0.) {
                m_turnMapping[pair.first][dsm::Direction::RIGHT] =
                    previousStreetId;  // right
              } else if (delta > 0.) {
                m_turnMapping[pair.first][dsm::Direction::LEFT] =
                    previousStreetId;  // left
              } else {
                m_turnMapping[pair.first][dsm::Direction::STRAIGHT] =
                    previousStreetId;  // straight
              }
            } else {
              m_turnMapping[pair.first][dsm::Direction::UTURN] = previousStreetId;  // U
            }
          }
        });
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_updatePath(std::unique_ptr<Itinerary> const& pItinerary) {
    if (m_bCacheEnabled) {
      auto const& file = std::format("{}it{}.adj", g_cacheFolder, pItinerary->id());
      if (std::filesystem::exists(file)) {
        pItinerary->setPath(AdjacencyMatrix(file));
        Logger::debug(
            std::format("Loaded cached path for itinerary {}", pItinerary->id()));
        return;
      }
    }
    auto const destinationID = pItinerary->destination();
    std::vector<DijkstraResult> shortestPaths(this->graph().nNodes());
    tbb::parallel_for_each(
        this->graph().nodes().cbegin(),
        this->graph().nodes().cend(),
        [this, &shortestPaths, &destinationID](auto const& it) -> void {
          auto const nodeId{it.first};
          if (nodeId == destinationID) {
            shortestPaths[nodeId] = DijkstraResult{};
          } else {
            auto result =
                this->graph().shortestPath(nodeId, destinationID, m_weightFunction);
            if (result.has_value()) {
              shortestPaths[nodeId] = *result;
            } else {
              Logger::warning(std::format(
                  "No path found from node {} to node {}", nodeId, destinationID));
              shortestPaths[nodeId] = DijkstraResult{};
            }
          }
        });
    AdjacencyMatrix path;
    // cycle over the nodes
    for (const auto& [nodeId, node] : this->graph().nodes()) {
      if (nodeId == destinationID) {
        continue;
      }
      // save the minimum distance between i and the destination
      const auto minDistance{shortestPaths[nodeId].distance()};
      if (minDistance < 0.) {
        continue;
      }
      auto const& row{this->graph().adjacencyMatrix().getRow(nodeId)};
      for (const auto nextNodeId : row) {
        if (nextNodeId == destinationID) {
          if (std::abs(m_weightFunction(&this->graph(), nodeId, nextNodeId) -
                       minDistance) <
              m_weightTreshold)  // 1 meter tolerance between shortest paths
          {
            path.insert(nodeId, nextNodeId);
          } else {
            Logger::debug(
                std::format("Found a path from {} to {} which differs for more than {} "
                            "unit(s) from the shortest one.",
                            nodeId,
                            destinationID,
                            m_weightTreshold));
          }
          continue;
        }
        auto const distance{shortestPaths[nextNodeId].distance()};
        if (distance < 0.) {
          continue;
        }
        if (std::find(shortestPaths[nextNodeId].path().cbegin(),
                      shortestPaths[nextNodeId].path().cend(),
                      nodeId) != shortestPaths[nextNodeId].path().cend()) {
          continue;
        }
        bool const bIsMinDistance{
            std::abs(m_weightFunction(&this->graph(), nodeId, nextNodeId) + distance -
                     minDistance) <
            m_weightTreshold};  // 1 meter tolerance between shortest paths
        if (bIsMinDistance) {
          path.insert(nodeId, nextNodeId);
        } else {
          Logger::debug(
              std::format("Found a path from {} to {} which differs for more than {} "
                          "unit(s) from the shortest one.",
                          nodeId,
                          destinationID,
                          m_weightTreshold));
        }
      }
    }

    if (path.empty()) {
      Logger::error(
          std::format("Path with id {} and destination {} is empty. Please check the "
                      "adjacency matrix.",
                      pItinerary->id(),
                      pItinerary->destination()));
    }

    pItinerary->setPath(path);
    if (m_bCacheEnabled) {
      pItinerary->path()->save(
          std::format("{}it{}.adj", g_cacheFolder, pItinerary->id()));
      Logger::debug(
          std::format("Saved path in cache for itinerary {}", pItinerary->id()));
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Id RoadDynamics<delay_t>::m_nextStreetId(std::unique_ptr<Agent> const& pAgent,
                                           Id nodeId,
                                           std::optional<Id> streetId) {
    auto possibleMoves = this->graph().adjacencyMatrix().getRow(nodeId);

    std::set<Id> forbiddenStreetIds;
    if (streetId.has_value()) {
      auto const& pStreet{this->graph().edge(*streetId)};
      forbiddenStreetIds = pStreet->forbiddenTurns();
      // Avoid U-TURNS, if possible
      if (!(this->graph().node(nodeId)->isRoundabout()) &&
          (possibleMoves.size() > forbiddenStreetIds.size() + 1)) {
        auto const& pOppositeStreet{this->graph().oppositeStreet(*streetId)};
        if (pOppositeStreet) {
          forbiddenStreetIds.insert(pOppositeStreet->get()->id());
        }
      }
    }
    // Exclude FORBIDDEN turns
    for (auto const& forbiddenStreetId : forbiddenStreetIds) {
      auto const& pForbiddenStreet{this->graph().edge(forbiddenStreetId)};
      // if possible moves contains the forbidden street, remove it
      auto it = std::find(
          possibleMoves.begin(), possibleMoves.end(), pForbiddenStreet->target());
      if (it != possibleMoves.end()) {
        possibleMoves.erase(it);
      }
    }

    if (!pAgent->isRandom()) {
      std::vector<Id> newPossibleMoves;
      std::uniform_real_distribution<double> uniformDist{0., 1.};
      if (!(this->itineraries().empty())) {
        if (!(m_errorProbability.has_value() &&
              uniformDist(this->m_generator) < m_errorProbability)) {
          const auto& it = this->itineraries().at(pAgent->itineraryId());
          if (it->destination() != nodeId) {
            newPossibleMoves = it->path()->getRow(nodeId);
          }
        }
        for (auto const& forbiddenStreetId : forbiddenStreetIds) {
          auto const& pForbiddenStreet{this->graph().edge(forbiddenStreetId)};
          // if possible moves contains the forbidden street, remove it
          auto it = std::find(newPossibleMoves.begin(),
                              newPossibleMoves.end(),
                              pForbiddenStreet->target());
          if (it != newPossibleMoves.end()) {
            newPossibleMoves.erase(it);
          }
        }
        if (!newPossibleMoves.empty()) {
          possibleMoves = newPossibleMoves;
        }
      }
    }

    assert(!possibleMoves.empty());

    if (possibleMoves.size() == 1) {
      return nodeId * this->graph().nNodes() + possibleMoves[0];
    }
    std::uniform_int_distribution<Size> moveDist{
        0, static_cast<Size>(possibleMoves.size() - 1)};
    return nodeId * this->graph().nNodes() + possibleMoves[moveDist(this->m_generator)];
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
    auto const& transportCapacity{pStreet->transportCapacity()};
    auto const nLanes = pStreet->nLanes();
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    for (auto i = 0; i < std::ceil(transportCapacity); ++i) {
      if (pStreet->isStochastic() &&
          (uniformDist(this->m_generator) >
           dynamic_cast<StochasticStreet&>(*pStreet).flowRate())) {
        continue;
      }
      if (i == std::ceil(transportCapacity) - 1) {
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          continue;
        }
      }
      for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
        if (pStreet->queue(queueIndex).empty()) {
          continue;
        }
        // Logger::debug("Taking temp agent");
        auto const& pAgentTemp{pStreet->queue(queueIndex).front()};
        if (pAgentTemp->freeTime() > this->time()) {
          // Logger::debug("Skipping due to time");
          continue;
        }
        {
          auto const timeDiff{this->time() - pAgentTemp->freeTime()};
          if (timeDiff > 120) {
            Logger::warning(std::format(
                "Time {} - Agent currently on {} ({} -> {}), targetting {} ({} turn - "
                "Traffic "
                "Light? {}), has been still for more than 120 seconds ({} seconds)",
                this->time(),
                pStreet->id(),
                pStreet->source(),
                pStreet->target(),
                pAgentTemp->nextStreetId().value_or(-1),
                directionToString.at(pStreet->laneMapping().at(queueIndex)),
                this->graph().node(pStreet->target())->isTrafficLight(),
                timeDiff));
          }
        }
        pAgentTemp->setSpeed(0.);
        const auto& destinationNode{this->graph().node(pStreet->target())};
        if (destinationNode->isFull()) {
          Logger::debug("Skipping due to space");
          continue;
        }
        if (destinationNode->isTrafficLight()) {
          auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
          auto const direction{pStreet->laneMapping().at(queueIndex)};
          if (!tl.isGreen(pStreet->id(), direction)) {
            Logger::debug(
                std::format("Skipping due to red light on street {} and direction {}",
                            pStreet->id(),
                            directionToString.at(direction)));
            continue;
          }
          Logger::debug(std::format("Green light on street {} and direction {}",
                                    pStreet->id(),
                                    directionToString.at(direction)));
        } else if (destinationNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
          bool bCanPass{true};
          if (!intersection.streetPriorities().contains(pStreet->id())) {
            // I have to check if the agent has right of way
            auto const& inNeighbours{
                this->graph().adjacencyMatrix().getCol(destinationNode->id())};
            for (auto const& sourceId : inNeighbours) {
              auto const& streetId{sourceId * this->graph().nNodes() +
                                   destinationNode->id()};
              if (streetId == pStreet->id()) {
                continue;
              }
              auto const& pStreetTemp{this->graph().edge(streetId)};
              if (intersection.streetPriorities().contains(streetId)) {
                if (pStreetTemp->nExitingAgents() > 0) {
                  Logger::debug(std::format(
                      "Skipping agent emission from street {} -> {} due to right of way.",
                      pStreet->source(),
                      pStreet->target()));
                  bCanPass = false;
                  break;
                }
              }
            }
            if (!bCanPass) {
              continue;
            }
          }
        }
        bool bArrived{false};
        if (!(uniformDist(this->m_generator) < m_passageProbability.value_or(1.1))) {
          if (pAgentTemp->isRandom()) {
            bArrived = true;
          } else {
            continue;
          }
        }
        if (!pAgentTemp->isRandom()) {
          if (destinationNode->id() ==
              this->itineraries().at(pAgentTemp->itineraryId())->destination()) {
            bArrived = true;
          }
        }
        if (bArrived) {
          auto pAgent{pStreet->dequeue(queueIndex)};
          m_travelDTs.push_back(
              {pAgent->distance(),
               static_cast<double>(this->time() - pAgent->spawnTime())});
          if (reinsert_agents) {
            // reset Agent's values
            pAgent->reset(this->time());
            this->addAgent(std::move(pAgent));
          }
          continue;
        }
        auto const& nextStreet{this->graph().edge(pAgentTemp->nextStreetId().value())};
        if (nextStreet->isFull()) {
          continue;
        }
        auto pAgent{pStreet->dequeue(queueIndex)};
        assert(destinationNode->id() == nextStreet->source());
        if (destinationNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
          auto const delta{nextStreet->deltaAngle(pStreet->angle())};
          // m_increaseTurnCounts(pStreet->id(), delta);
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
          return;
        }
      }
      if (pNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pNode);
        if (intersection.agents().empty()) {
          // Logger::debug(std::format("No agents on node {}", pNode->id()));
          return;
        }
        for (auto it{intersection.agents().begin()}; it != intersection.agents().end();) {
          auto& pAgent{it->second};
          auto const& nextStreet{this->graph().edge(pAgent->nextStreetId().value())};
          if (nextStreet->isFull()) {
            Logger::debug(std::format("Next street {} is full", nextStreet->id()));
            if (m_forcePriorities) {
              return;
            }
            ++it;
            continue;
          }
          pAgent->setStreetId();
          this->setAgentSpeed(pAgent);
          pAgent->setFreeTime(this->time() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
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
          if (pAgentTemp->streetId().has_value()) {
            const auto streetId = pAgentTemp->streetId().value();
            auto delta = nextStreet->angle() - this->graph().edge(streetId)->angle();
            if (delta > std::numbers::pi) {
              delta -= 2 * std::numbers::pi;
            } else if (delta < -std::numbers::pi) {
              delta += 2 * std::numbers::pi;
            }
            m_increaseTurnCounts(streetId, delta);
          }
          auto pAgent{roundabout.dequeue()};
          pAgent->setStreetId();
          this->setAgentSpeed(pAgent);
          pAgent->setFreeTime(this->time() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
          nextStreet->addAgent(std::move(pAgent));
        } else {
          return;
        }
      }
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::m_evolveAgent(std::unique_ptr<Agent> const& pAgent) {
    // The "cost" of enqueuing is one time unit, so we consider it as passed
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
  void RoadDynamics<delay_t>::setDestinationNodes(
      std::initializer_list<Id> destinationNodes, bool updatePaths) {
    std::for_each(
        destinationNodes.begin(),
        destinationNodes.end(),
        [this](auto const& nodeId) -> void { this->addItinerary(nodeId, nodeId); });
    if (updatePaths) {
      this->updatePaths();
    }
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_convertible_v<typename TContainer::value_type, Id>)
  void RoadDynamics<delay_t>::setDestinationNodes(TContainer const& destinationNodes,
                                                  bool updatePaths) {
    std::for_each(
        destinationNodes.begin(),
        destinationNodes.end(),
        [this](auto const& nodeId) -> void { this->addItinerary(nodeId, nodeId); });
    if (updatePaths) {
      this->updatePaths();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::updatePaths() {
    Logger::debug("Init updating paths...");
    tbb::parallel_for_each(
        this->itineraries().cbegin(),
        this->itineraries().cend(),
        [this](auto const& pair) -> void { this->m_updatePath(pair.second); });
    Logger::debug("End updating paths.");
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
        0, static_cast<Size>(this->graph().nEdges() - 1)};
    for (Size i{0}; i < nAgents; ++i) {
      if (randomItinerary) {
        auto itineraryIt{this->itineraries().cbegin()};
        std::advance(itineraryIt, itineraryDist(this->m_generator));
        itineraryId = itineraryIt->first;
      }
      Id streetId{0};
      do {
        auto streetIt = this->graph().edges().begin();
        Size step = streetDist(this->m_generator);
        std::advance(streetIt, step);
        streetId = streetIt->first;
      } while (this->graph().edge(streetId)->isFull() &&
               this->nAgents() < this->graph().maxCapacity());
      const auto& street{this->graph().edge(streetId)};
      auto pAgent{std::make_unique<Agent>(this->time(), itineraryId, street->source())};
      pAgent->setStreetId(streetId);
      this->setAgentSpeed(pAgent);
      pAgent->setFreeTime(this->time() + std::ceil(street->length() / pAgent->speed()));
      street->addAgent(std::move(pAgent));
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename TContainer>
    requires(std::is_same_v<TContainer, std::unordered_map<Id, double>> ||
             std::is_same_v<TContainer, std::map<Id, double>>)
  void RoadDynamics<delay_t>::addAgentsRandomly(
      Size nAgents,
      const TContainer& src_weights,
      const TContainer& dst_weights,
      const std::variant<std::monostate, size_t, double> minNodeDistance) {
    auto const& nSources{src_weights.size()};
    auto const& nDestinations{dst_weights.size()};
    Logger::debug(
        std::format("Init addAgentsRandomly for {} agents from {} nodes to {} nodes.",
                    nAgents,
                    nSources,
                    dst_weights.size()));
    if (nSources == 1 && nDestinations == 1 &&
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
    Logger::debug(std::format("Adding {} agents at time {}.", nAgents, this->time()));
    while (nAgents > 0) {
      Id srcId{0}, dstId{0};
      if (nDestinations == 1) {
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
      if (nSources > 1) {
        dstId = srcId;
      }
      while (dstId == srcId) {
        dRand = dstUniformDist(this->m_generator);
        sum = 0.;
        for (const auto& [id, weight] : dst_weights) {
          // if the node is at a minimum distance from the destination, skip it
          if (this->itineraries().at(id)->path()->getRow(srcId).empty()) {
            continue;
          }
          if (std::holds_alternative<size_t>(minNodeDistance)) {
            auto const minDistance{std::get<size_t>(minNodeDistance)};
            if (this->graph().shortestPath(srcId, id).value().path().size() <
                minDistance) {
              continue;
            }
          } else if (std::holds_alternative<double>(minNodeDistance)) {
            auto const minDistance{std::get<double>(minNodeDistance)};
            if (this->graph()
                    .shortestPath(srcId, id, weight_functions::streetLength)
                    .value()
                    .distance() < minDistance) {
              Logger::debug(
                  std::format("Skipping node {} because the distance from the source "
                              "is less than {}",
                              id,
                              minDistance));
              continue;
            }
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
      this->addAgent(itineraryIt->first, srcId);
      --nAgents;
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgentsRandomly(
      Size nAgents, const std::variant<std::monostate, size_t, double> minNodeDistance) {
    std::unordered_map<Id, double> src_weights, dst_weights;
    for (auto const& id : this->graph().inputNodes()) {
      src_weights[id] = 1.;
    }
    for (auto const& id : this->graph().outputNodes()) {
      dst_weights[id] = 1.;
    }
    addAgentsRandomly(nAgents, src_weights, dst_weights, minNodeDistance);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::addAgent(std::unique_ptr<Agent> pAgent) {
    m_agents.push_back(std::move(pAgent));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, Time, TArgs...>)
  void RoadDynamics<delay_t>::addAgent(TArgs&&... args) {
    addAgent(std::make_unique<Agent>(this->time(), std::forward<TArgs>(args)...));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  template <typename... TArgs>
    requires(std::is_constructible_v<Agent, Time, TArgs...>)
  void RoadDynamics<delay_t>::addAgents(Size nAgents, TArgs&&... args) {
    for (size_t i{0}; i < nAgents; ++i) {
      addAgent(std::make_unique<Agent>(this->time(), std::forward<TArgs>(args)...));
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
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Itinerary with id {} already exists.", itinerary->id())));
    }
    if (!this->graph().nodes().contains(itinerary->destination())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(std::format(
          "Destination node with id {} not found", itinerary->destination())));
    }
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::evolve(bool reinsert_agents) {
    Logger::debug(std::format("Init evolve at time {}", this->time()));
    // move the first agent of each street queue, if possible, putting it in the next node
    bool const bUpdateData =
        m_dataUpdatePeriod.has_value() && this->time() % m_dataUpdatePeriod.value() == 0;
    auto const N{this->graph().nNodes()};
    auto const numNodes{this->graph().nNodes()};
    const auto& nodes =
        this->graph().nodes();  // assuming a container with contiguous indices
    const unsigned int concurrency = std::thread::hardware_concurrency();
    // Calculate a grain size to partition the nodes into roughly "concurrency" blocks
    const size_t grainSize = std::max(size_t(1), numNodes / concurrency);
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
              auto const& pNode = nodes.at(i);
              for (auto const& sourceId :
                   this->graph().adjacencyMatrix().getCol(pNode->id())) {
                auto const streetId{sourceId * N + pNode->id()};
                auto const& pStreet{this->graph().edge(streetId)};
                if (bUpdateData && pNode->isTrafficLight()) {
                  if (!m_queuesAtTrafficLights.contains(streetId)) {
                    auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                    for (auto const& [streetId, pair] : tl.cycles()) {
                      for (auto const& [direction, cycle] : pair) {
                        m_queuesAtTrafficLights[streetId].emplace(direction, 0.);
                      }
                    }
                  }
                  for (auto& [direction, value] : m_queuesAtTrafficLights.at(streetId)) {
                    value += pStreet->nExitingAgents(direction, true);
                  }
                }
                m_evolveStreet(pStreet, reinsert_agents);

                while (!pStreet->movingAgents().empty()) {
                  auto const& pAgent{pStreet->movingAgents().top()};
                  if (pAgent->freeTime() < this->time()) {
                    break;
                  }
                  pAgent->setSpeed(0.);
                  auto const nLanes = pStreet->nLanes();
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
                    std::uniform_int_distribution<size_t> laneDist{
                        0, static_cast<size_t>(nLanes - 1)};
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
                  auto const deltaAngle{pNextStreet->deltaAngle(pStreet->angle())};
                  if (std::abs(deltaAngle) < std::numbers::pi) {
                    // Lanes are counted as 0 is the far right lane
                    if (std::abs(deltaAngle) < std::numbers::pi / 8) {
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
                      auto const sum =
                          std::accumulate(weights.begin(), weights.end(), 0.);
                      for (auto& w : weights) {
                        w /= sum;
                      }
                      std::discrete_distribution<size_t> laneDist{weights.begin(),
                                                                  weights.end()};
                      pStreet->enqueue(laneDist(this->m_generator));
                    } else if (deltaAngle < 0.) {    // Right
                      pStreet->enqueue(0);           // Always the first lane
                    } else {                         // Left (deltaAngle > 0.)
                      pStreet->enqueue(nLanes - 1);  // Always the last lane
                    }
                  } else {                         // U turn
                    pStreet->enqueue(nLanes - 1);  // Always the last lane
                  }
                }
              }
            }
          });
    });
    Logger::debug("Pre-nodes");
    // Move transport capacity agents from each node
    this->m_taskArena.execute([&] {
      tbb::parallel_for(tbb::blocked_range<size_t>(0, numNodes, grainSize),
                        [&](const tbb::blocked_range<size_t>& range) {
                          for (size_t i = range.begin(); i != range.end(); ++i) {
                            const auto& pNode = nodes.at(i);
                            m_evolveNode(pNode);
                            if (pNode->isTrafficLight()) {
                              auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                              ++tl;
                            }
                          }
                        });
    });
    // cycle over agents and update their times
    std::uniform_int_distribution<Id> nodeDist{
        0, static_cast<Id>(this->graph().nNodes() - 1)};
    Logger::debug("Pre-agents");
    for (auto itAgent{m_agents.begin()}; itAgent != m_agents.end();) {
      auto& pAgent{*itAgent};
      if (!pAgent->srcNodeId().has_value()) {
        pAgent->setSrcNodeId(nodeDist(this->m_generator));
      }
      auto const& srcNode{this->graph().node(pAgent->srcNodeId().value())};
      if (srcNode->isFull()) {
        ++itAgent;
        continue;
      }
      if (!pAgent->nextStreetId().has_value()) {
        pAgent->setNextStreetId(
            this->m_nextStreetId(pAgent, srcNode->id(), pAgent->streetId()));
      }
      auto const& nextStreet{
          this->graph().edge(pAgent->nextStreetId().value())};  // next street
      if (nextStreet->isFull()) {
        ++itAgent;
        continue;
      }
      assert(srcNode->id() == nextStreet->source());
      if (srcNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*srcNode);
        intersection.addAgent(0., std::move(pAgent));
      } else if (srcNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*srcNode);
        roundabout.enqueue(std::move(pAgent));
      }
      itAgent = m_agents.erase(itAgent);
    }
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
          this->time(),
          beta);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);

      auto const& inNeighbours{this->graph().adjacencyMatrix().getCol(nodeId)};

      // Default is RIGHTANDSTRAIGHT - LEFT phases for both priority and non-priority
      std::array<double, 2> inputPrioritySum{0., 0.}, inputNonPrioritySum{0., 0.};
      bool isPrioritySinglePhase{false}, isNonPrioritySinglePhase{false};

      for (const auto& sourceId : inNeighbours) {
        auto const streetId = sourceId * this->graph().nNodes() + nodeId;
        if (tl.cycles().at(streetId).contains(Direction::ANY)) {
          tl.streetPriorities().contains(streetId) ? isPrioritySinglePhase = true
                                                   : isNonPrioritySinglePhase = true;
        }
      }
      if (isPrioritySinglePhase && logStream.has_value()) {
        *logStream << std::format("\tFound a single phase for priority streets.\n");
      }
      if (isNonPrioritySinglePhase && logStream.has_value()) {
        *logStream << std::format("\tFound a single phase for non-priority streets.\n");
      }

      for (const auto& sourceId : inNeighbours) {
        auto const streetId = sourceId * this->graph().nNodes() + nodeId;
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
                  *logStream << std::format("\tFree cycle ({}) for {} -> {}: {} {}\n",
                                            directionToString[direction],
                                            pStreet->source(),
                                            pStreet->target(),
                                            freecycle.greenTime(),
                                            freecycle.phase());
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
        std::string logMsg =
            std::format("\tNew cycles for TL {} ({}):\n", tl.id(), tl.cycleTime());
        for (auto const& [streetId, pair] : tl.cycles()) {
          auto const& pStreet{this->graph().edge(streetId)};
          logMsg +=
              std::format("\t\tStreet {} -> {}: ", pStreet->source(), pStreet->target());
          for (auto const& [direction, cycle] : pair) {
            logMsg += std::format("{}= ({} {}) ",
                                  directionToString[direction],
                                  cycle.greenTime(),
                                  cycle.phase());
          }
          logMsg += '\n';
        }
        *logStream << logMsg << '\n';
      }
    }
    if (logStream.has_value()) {
      *logStream << std::format("End Traffic Lights optimization - Time {}\n",
                                this->time());
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::optimizeTrafficLights(
      double const threshold,
      TrafficLightOptimization const optimizationType,
      const std::string& logFile) {
    std::optional<std::ofstream> logStream;
    if (!logFile.empty()) {
      logStream.emplace(logFile, std::ios::app);
      if (!logStream->is_open()) {
        Logger::error(std::format("Could not open log file: {}", logFile));
      }
    }
    if (optimizationType == TrafficLightOptimization::SINGLE_TAIL) {
      this->m_trafficlightSingleTailOptimizer(threshold, logStream);
    } else if (optimizationType == TrafficLightOptimization::DOUBLE_TAIL) {
      if (threshold < 0) {
        Logger::error(std::format("The threshold parameter ({}) must be greater than 0.",
                                  threshold));
      }
      auto const nCycles{static_cast<double>(this->time() - m_previousOptimizationTime) /
                         m_dataUpdatePeriod.value()};
      for (const auto& [nodeId, pNode] : this->graph().nodes()) {
        if (!pNode->isTrafficLight()) {
          continue;
        }
        if (logStream.has_value()) {
          *logStream << std::format("\tTraffic Light {}\n", nodeId);
        }
        auto& tl = dynamic_cast<TrafficLight&>(*pNode);
        auto const& streetPriorities = tl.streetPriorities();
        auto const meanGreenFraction{tl.meanGreenTime(true) / tl.cycleTime()};
        auto const meanRedFraction{tl.meanGreenTime(false) / tl.cycleTime()};

        double inputGreenSum{0.}, inputRedSum{0.};
        auto const N{this->graph().nNodes()};
        auto column = this->graph().adjacencyMatrix().getCol(nodeId);
        for (const auto& sourceId : column) {
          auto const streetId = sourceId * N + nodeId;
          auto const& pStreet{this->graph().edge(streetId)};
          if (streetPriorities.contains(streetId)) {
            for (auto const& pair : m_queuesAtTrafficLights.at(streetId)) {
              inputGreenSum += pair.second / pStreet->nLanes();
            }
          } else {
            for (auto const& pair : m_queuesAtTrafficLights.at(streetId)) {
              inputRedSum += pair.second / pStreet->nLanes();
            }
          }
        }
        inputGreenSum /= meanGreenFraction;
        inputRedSum /= meanRedFraction;
        auto const inputDifference{(inputGreenSum - inputRedSum) / nCycles};
        delay_t const delta = std::round(std::abs(inputDifference) / column.size());
        auto const greenTime = tl.minGreenTime(true);
        auto const redTime = tl.minGreenTime(false);
        // If the difference is not less than the threshold
        //    - Check that the incoming streets have a density less than the mean one (eventually + tolerance): I want to avoid being into the cluster, better to be out or on the border
        //    - If the previous check fails, do nothing
        double outputGreenSum{0.}, outputRedSum{0.};
        for (auto const& targetId : this->graph().adjacencyMatrix().getRow(nodeId)) {
          auto const streetId = nodeId * N + targetId;
          if (!m_queuesAtTrafficLights.contains(streetId)) {
            continue;
          }
          auto const& pStreet{this->graph().edge(streetId)};
          if (streetPriorities.contains(streetId)) {
            for (auto const& pair : m_queuesAtTrafficLights.at(streetId)) {
              outputGreenSum += pair.second / pStreet->nLanes();
            }
          } else {
            for (auto const& pair : m_queuesAtTrafficLights.at(streetId)) {
              outputGreenSum += pair.second / pStreet->nLanes();
            }
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
    for (auto& [streetId, pair] : m_queuesAtTrafficLights) {
      for (auto& [direction, value] : pair) {
        value = 0.;
      }
    }
    m_previousOptimizationTime = this->time();
    if (logStream.has_value()) {
      logStream->close();
    }
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  size_t RoadDynamics<delay_t>::nAgents() const {
    auto nAgents{m_agents.size()};
    Logger::debug(std::format("Number of agents: {}", nAgents));
    for (const auto& [nodeId, pNode] : this->graph().nodes()) {
      if (pNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pNode);
        nAgents += intersection.agents().size();
      } else if (pNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
        nAgents += roundabout.agents().size();
      }
      Logger::debug(std::format("Number of agents: {}", nAgents));
    }
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      nAgents += pStreet->nAgents();
    }
    Logger::debug(std::format("Number of agents: {}", nAgents));
    return nAgents;
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
  Measurement<double> RoadDynamics<delay_t>::agentMeanSpeed() const {
    std::vector<double> speeds;
    // if (!this->agents().empty()) {
    //   speeds.reserve(this->nAgents());
    //   for (const auto& [agentId, agent] : this->agents()) {
    //     speeds.push_back(agent->speed());
    //   }
    // }
    return Measurement<double>(speeds);
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
    for (const auto& [streetId, street] : this->graph().edges()) {
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
    for (const auto& [streetId, street] : this->graph().edges()) {
      if (above && (street->density(true) > threshold)) {
        speeds.push_back(streetMeanSpeed(streetId));
      } else if (!above && (street->density(true) < threshold)) {
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
      for (const auto& [streetId, street] : this->graph().edges()) {
        densities.push_back(street->density(true));
      }
    } else {
      double sum{0.};
      for (const auto& [streetId, street] : this->graph().edges()) {
        densities.push_back(street->density(false) * street->length());
        sum += street->length();
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
    for (const auto& [streetId, street] : this->graph().edges()) {
      flows.push_back(street->density() * this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::streetMeanFlow(double threshold,
                                                            bool above) const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, street] : this->graph().edges()) {
      if (above && (street->density(true) > threshold)) {
        flows.push_back(street->density() * this->streetMeanSpeed(streetId));
      } else if (!above && (street->density(true) < threshold)) {
        flows.push_back(street->density() * this->streetMeanSpeed(streetId));
      }
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::meanSpireInputFlow(bool resetValue) {
    auto deltaTime{this->time() - m_previousSpireTime};
    if (deltaTime == 0) {
      return Measurement(0., 0.);
    }
    m_previousSpireTime = this->time();
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, street] : this->graph().edges()) {
      if (street->isSpire()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        flows.push_back(static_cast<double>(spire.inputCounts(resetValue)) / deltaTime);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  Measurement<double> RoadDynamics<delay_t>::meanSpireOutputFlow(bool resetValue) {
    auto deltaTime{this->time() - m_previousSpireTime};
    if (deltaTime == 0) {
      return Measurement(0., 0.);
    }
    m_previousSpireTime = this->time();
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (auto const& [streetId, street] : this->graph().edges()) {
      if (street->isSpire()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        flows.push_back(static_cast<double>(spire.outputCounts(resetValue)) / deltaTime);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveStreetDensities(const std::string& filename,
                                                  bool normalized,
                                                  char const separator) const {
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
      file << "time";
      for (auto const& [streetId, _] : this->graph().edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->graph().edges()) {
      // keep 2 decimal digits;
      file << separator << std::scientific << std::setprecision(2)
           << pStreet->density(normalized);
    }
    file << std::endl;
    file.close();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveInputStreetCounts(const std::string& filename,
                                                    bool reset,
                                                    char const separator) {
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
      file << "time";
      for (auto const& [streetId, _] : this->graph().edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->graph().edges()) {
      int value{0};
      if (pStreet->isSpire()) {
        if (pStreet->isStochastic()) {
          value = dynamic_cast<StochasticSpireStreet&>(*pStreet).inputCounts(reset);
        } else {
          value = dynamic_cast<SpireStreet&>(*pStreet).inputCounts(reset);
        }
      }
      file << separator << value;
    }
    file << std::endl;
    file.close();
  }
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveOutputStreetCounts(const std::string& filename,
                                                     bool reset,
                                                     char const separator) {
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
      file << "time";
      for (auto const& [streetId, pStreet] : this->graph().edges()) {
        if (!pStreet->isSpire()) {
          continue;
        }
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->graph().edges()) {
      int value{0};
      if (pStreet->isSpire()) {
        if (pStreet->isStochastic()) {
          value = dynamic_cast<StochasticSpireStreet&>(*pStreet).outputCounts(reset);
        } else {
          value = dynamic_cast<SpireStreet&>(*pStreet).outputCounts(reset);
        }
        file << separator << value;
      }
    }
    file << std::endl;
    file.close();
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
  template <typename delay_t>
    requires(is_numeric_v<delay_t>)
  void RoadDynamics<delay_t>::saveMacroscopicObservables(const std::string& filename,
                                                         char const separator) {
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
      file << "time;n_ghost_agents;n_agents;mean_speed;mean_speed_std;mean_density;mean_"
              "density_std;"
              "mean_flow;mean_flow_std;mean_flow_spires;mean_flow_spires_std;mean_"
              "traveltime;mean_traveltime_std;mean_traveldistance;mean_traveldistance_"
              "err;mean_travelspeed;mean_travelspeed_std\n";
    }
    file << this->time() << separator;
    file << m_agents.size() << separator;
    {
      std::vector<double> speeds, densities, flows, spireFlows;
      size_t nAgents{0};
      speeds.reserve(this->graph().nEdges());
      densities.reserve(this->graph().nEdges());
      flows.reserve(this->graph().nEdges());
      spireFlows.reserve(this->graph().nEdges());
      for (auto const& [streetId, street] : this->graph().edges()) {
        nAgents += street->nAgents();
        speeds.push_back(this->streetMeanSpeed(streetId));
        densities.push_back(street->density(true));
        flows.push_back(street->density(true) * speeds.back());
        if (street->isSpire()) {
          auto& spire = dynamic_cast<SpireStreet&>(*street);
          spireFlows.push_back(static_cast<double>(spire.inputCounts(true)) /
                               (this->time() - m_previousSpireTime));
        }
      }
      for (auto const& [nodeId, pNode] : this->graph().nodes()) {
        if (pNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*pNode);
          nAgents += intersection.agents().size();
        } else if (pNode->isRoundabout()) {
          auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
          nAgents += roundabout.agents().size();
        }
      }
      auto speed{Measurement<double>(speeds)};
      auto density{Measurement<double>(densities)};
      auto flow{Measurement<double>(flows)};
      auto spireFlow{Measurement<double>(spireFlows)};
      file << nAgents << separator;
      file << std::scientific << std::setprecision(2);
      file << speed.mean << separator << speed.std << separator;
      file << density.mean << separator << density.std << separator;
      file << flow.mean << separator << flow.std << separator;
      file << spireFlow.mean << separator << spireFlow.std << separator;
    }
    {
      std::vector<double> distances, times, speeds;
      distances.reserve(m_travelDTs.size());
      times.reserve(m_travelDTs.size());
      speeds.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        distances.push_back(distance);
        times.push_back(time);
        speeds.push_back(distance / time);
      }
      auto distance{Measurement<double>(distances)};
      auto time{Measurement<double>(times)};
      auto speed{Measurement<double>(speeds)};
      file << time.mean << separator << time.std << separator;
      file << distance.mean << separator << distance.std << separator;
      file << speed.mean << separator << speed.std << std::endl;
      m_travelDTs.clear();
    }
    m_travelDTs.clear();
    file.close();
  }
};  // namespace dsm
