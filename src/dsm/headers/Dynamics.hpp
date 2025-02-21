/// @file       /src/dsm/headers/Dynamics.hpp
/// @brief      Defines the Dynamics class.
///
/// @details    This file contains the definition of the Dynamics class.
///             The Dynamics class represents the dynamics of the network. It is templated by the type
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
#include <filesystem>
#include <functional>
#include <tbb/tbb.h>

#include "DijkstraWeights.hpp"
#include "Itinerary.hpp"
#include "RoadNetwork.hpp"
#include "SparseMatrix.hpp"
#include "../utility/TypeTraits/is_agent.hpp"
#include "../utility/TypeTraits/is_itinerary.hpp"
#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

static auto constexpr g_cacheFolder = "./.dsmcache/";

namespace dsm {
  /// @brief The Measurement struct represents the mean of a quantity and its standard deviation
  /// @tparam T The type of the quantity
  /// @param mean The mean
  /// @param std The standard deviation of the sample
  template <typename T>
  struct Measurement {
    T mean;
    T std;

    Measurement(T mean, T std) : mean{mean}, std{std} {}
    Measurement(std::span<T> data) {
      auto x_mean = static_cast<T>(0), x2_mean = static_cast<T>(0);
      if (data.empty()) {
        mean = static_cast<T>(0);
        std = static_cast<T>(0);
        return;
      }

      std::for_each(data.begin(), data.end(), [&x_mean, &x2_mean](auto value) -> void {
        x_mean += value;
        x2_mean += value * value;
      });
      mean = x_mean / data.size();
      std = std::sqrt(x2_mean / data.size() - mean * mean);
    }
  };

  /// @brief The Dynamics class represents the dynamics of the network.
  /// @tparam Id, The type of the graph's id. It must be an unsigned integral type.
  /// @tparam Size, The type of the graph's capacity. It must be an unsigned integral type.
  template <typename agent_t>
  class Dynamics {
  private:
    std::map<Id, std::unique_ptr<agent_t>> m_agents;
    std::unordered_map<Id, std::unique_ptr<Itinerary>> m_itineraries;

  protected:
    tbb::task_arena m_arena;

  private:
    std::function<double(const RoadNetwork*, Id, Id)> m_weightFunction;
    double m_weightTreshold;
    bool m_bCacheEnabled;

  protected:
    RoadNetwork m_graph;
    Time m_time, m_previousSpireTime;
    std::mt19937_64 m_generator;

    /// @brief Update the path of a single itinerary using Dijsktra's algorithm
    /// @param pItinerary An std::unique_prt to the itinerary
    void m_updatePath(const std::unique_ptr<Itinerary>& pItinerary) {
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
      std::vector<double> shortestDistances(m_graph.nNodes());
      tbb::parallel_for_each(
          m_graph.nodes().cbegin(),
          m_graph.nodes().cend(),
          [this, &shortestDistances, &destinationID](auto const& it) -> void {
            auto const nodeId{it.first};
            if (nodeId == destinationID) {
              shortestDistances[nodeId] = -1.;
            } else {
              auto result = m_graph.shortestPath(nodeId, destinationID, m_weightFunction);
              if (result.has_value()) {
                shortestDistances[nodeId] = result.value().distance();
              } else {
                Logger::warning(std::format(
                    "No path found from node {} to node {}", nodeId, destinationID));
                shortestDistances[nodeId] = -1.;
              }
            }
          });
      AdjacencyMatrix path;
      // cycle over the nodes
      for (const auto& [nodeId, node] : m_graph.nodes()) {
        if (nodeId == destinationID) {
          continue;
        }
        // save the minimum distance between i and the destination
        const auto minDistance{shortestDistances[nodeId]};
        if (minDistance < 0.) {
          continue;
        }
        auto const& row{m_graph.adjacencyMatrix().getRow(nodeId)};
        for (const auto nextNodeId : row) {
          if (nextNodeId == destinationID) {
            if (std::abs(m_weightFunction(&m_graph, nodeId, nextNodeId) - minDistance) <
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
          auto const distance{shortestDistances[nextNodeId]};
          if (distance < 0.) {
            continue;
          }
          bool const bIsMinDistance{
              std::abs(m_weightFunction(&m_graph, nodeId, nextNodeId) + distance -
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

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param useCache If true, the paths are cached (default is false)
    /// @param seed The seed for the random number generator (default is std::nullopt)
    /// @param weightFunction The weight function for the Dijkstra's algorithm (default is
    /// weight_functions::streetLength)
    /// @param weightTreshold The weight treshold for updating the paths (default is 1.)
    Dynamics(RoadNetwork& graph,
             bool useCache = false,
             std::optional<unsigned int> seed = std::nullopt,
             std::function<double(const RoadNetwork*, Id, Id)> weightFunction =
                 weight_functions::streetLength,
             double weightTreshold = 1.);

    virtual void setAgentSpeed(Size agentId) = 0;
    virtual void evolve(bool reinsert_agents = false) = 0;

    /// @brief Update the paths of the itineraries based on the actual travel times
    virtual void updatePaths();

    /// @brief Set the weight function for the Dijkstra's algorithm
    /// @param weightFunction A std::function returning a double value and taking as arguments a
    /// pointer to the graph, an id of a source node and an id of a target node (for the edge)
    /// @details The weight function must return the weight of the edge between the source and the
    /// target node. One can use the predefined weight functions in the DijkstraWeights.hpp file,
    /// like weight_functions::streetLength or weight_functions::streetTime.
    void setWeightFunction(
        std::function<double(const RoadNetwork*, Id, Id)> weightFunction);
    /// @brief Set the weight treshold for updating the paths
    /// @param weightTreshold The weight treshold
    /// @details If two paths differs only for a weight smaller than the treshold, the two paths are
    /// considered equivalent.
    void setWeightTreshold(double weightTreshold) { m_weightTreshold = weightTreshold; }

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

    /// @brief Add an agent to the simulation
    /// @param agent std::unique_ptr to the agent
    void addAgent(std::unique_ptr<agent_t> agent);

    template <typename... TArgs>
      requires(std::is_constructible_v<agent_t, TArgs...>)
    void addAgent(TArgs&&... args);

    template <typename... TArgs>
      requires(std::is_constructible_v<agent_t, Id, TArgs...>)
    void addAgents(Size nAgents, TArgs&&... args);

    /// @brief Add a pack of agents to the simulation
    /// @param agents Parameter pack of agents
    template <typename... Tn>
      requires(is_agent_v<Tn> && ...)
    void addAgents(Tn... agents);
    /// @brief Add a pack of agents to the simulation
    /// @param agent An agent
    /// @param agents Parameter pack of agents
    template <typename T1, typename... Tn>
      requires(is_agent_v<T1> && (is_agent_v<Tn> && ...))
    void addAgents(T1 agent, Tn... agents);
    /// @brief Add a set of agents to the simulation
    /// @param agents Generic container of agents, represented by an std::span
    void addAgents(std::span<agent_t> agents);

    /// @brief Remove an agent from the simulation
    /// @param agentId the id of the agent to remove
    void removeAgent(Size agentId);
    template <typename T1, typename... Tn>
      requires(std::is_convertible_v<T1, Id> && (std::is_convertible_v<Tn, Size> && ...))
    /// @brief Remove a pack of agents from the simulation
    /// @param id the id of the first agent to remove
    /// @param ids the pack of ides of the agents to remove
    void removeAgents(T1 id, Tn... ids);

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

    /// @brief Reset the simulation time
    void resetTime();

    /// @brief Get the graph
    /// @return const RoadNetwork&, The graph
    const RoadNetwork& graph() const { return m_graph; };
    /// @brief Get the itineraries
    /// @return const std::unordered_map<Id, Itinerary>&, The itineraries
    const std::unordered_map<Id, std::unique_ptr<Itinerary>>& itineraries() const {
      return m_itineraries;
    }
    /// @brief Get the agents
    /// @return const std::unordered_map<Id, Agent<Id>>&, The agents
    const std::map<Id, std::unique_ptr<agent_t>>& agents() const { return m_agents; }
    /// @brief Get the number of agents currently in the simulation
    /// @return Size The number of agents
    Size nAgents() const { return m_agents.size(); }
    /// @brief Get the time
    /// @return Time The time
    Time time() const { return m_time; }

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
    /// @brief Get the mean travel time of the agents in \f$s\f$
    /// @param clearData If true, the travel times are cleared after the computation
    /// @return Measurement<double> The mean travel time of the agents and the standard
    Measurement<double> meanTravelTime(bool clearData = false);

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
  };

  template <typename agent_t>
  Dynamics<agent_t>::Dynamics(
      RoadNetwork& graph,
      bool useCache,
      std::optional<unsigned int> seed,
      std::function<double(const RoadNetwork*, Id, Id)> weightFunction,
      double weightTreshold)
      : m_weightFunction{weightFunction},
        m_weightTreshold{weightTreshold},
        m_bCacheEnabled{useCache},
        m_graph{std::move(graph)},
        m_time{0},
        m_previousSpireTime{0},
        m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(seed.value());
    }
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(g_cacheFolder)) {
        std::filesystem::create_directory(g_cacheFolder);
      }
      Logger::info(std::format("Cache enabled (default folder is {})", g_cacheFolder));
    }
    m_arena.initialize(static_cast<int>(std::thread::hardware_concurrency()));
    for (const auto& nodeId : this->m_graph.outputNodes()) {
      addItinerary(nodeId, nodeId);
    }
    updatePaths();
  }

  template <typename agent_t>
  void Dynamics<agent_t>::updatePaths() {
    Logger::debug("Init updating paths...");
    tbb::parallel_for_each(
        m_itineraries.cbegin(), m_itineraries.cend(), [this](auto const& pair) -> void {
          this->m_updatePath(pair.second);
        });
    Logger::debug("End updating paths.");
  }

  template <typename agent_t>
  void Dynamics<agent_t>::setWeightFunction(
      std::function<double(const RoadNetwork*, Id, Id)> weightFunction) {
    m_weightFunction = weightFunction;
  }

  template <typename agent_t>
  void Dynamics<agent_t>::setDestinationNodes(std::initializer_list<Id> destinationNodes,
                                              bool updatePaths) {
    std::for_each(
        destinationNodes.begin(),
        destinationNodes.end(),
        [this](auto const& nodeId) -> void { this->addItinerary(nodeId, nodeId); });
    if (updatePaths) {
      this->updatePaths();
    }
  }
  template <typename agent_t>
  template <typename TContainer>
    requires(std::is_convertible_v<typename TContainer::value_type, Id>)
  void Dynamics<agent_t>::setDestinationNodes(TContainer const& destinationNodes,
                                              bool updatePaths) {
    std::for_each(
        destinationNodes.begin(),
        destinationNodes.end(),
        [this](auto const& nodeId) -> void { this->addItinerary(nodeId, nodeId); });
    if (updatePaths) {
      this->updatePaths();
    }
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addAgent(std::unique_ptr<agent_t> agent) {
    if (m_agents.size() + 1 > m_graph.maxCapacity()) {
      throw std::overflow_error(Logger::buildExceptionMessage(std::format(
          "RoadNetwork is already holding the max possible number of agents ({})",
          m_graph.maxCapacity())));
    }
    if (m_agents.contains(agent->id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Agent with id {} already exists.", agent->id())));
    }
    m_agents.emplace(agent->id(), std::move(agent));
    // Logger::info(std::format("Added agent with id {} from node {} to node {}",
    //                           m_agents.rbegin()->first,
    //                           m_agents.rbegin()->second->srcNodeId().value_or(-1),
    //                           m_agents.rbegin()->second->itineraryId()));
  }

  template <typename agent_t>
  template <typename... TArgs>
    requires(std::is_constructible_v<agent_t, TArgs...>)
  void Dynamics<agent_t>::addAgent(TArgs&&... args) {
    addAgent(std::make_unique<agent_t>(std::forward<TArgs>(args)...));
  }

  template <typename agent_t>
  template <typename... TArgs>
    requires(std::is_constructible_v<agent_t, Id, TArgs...>)
  void Dynamics<agent_t>::addAgents(Size nAgents, TArgs&&... args) {
    Id agentId{0};
    if (!m_agents.empty()) {
      agentId = m_agents.rbegin()->first + 1;
    }
    for (size_t i{0}; i < nAgents; ++i, ++agentId) {
      addAgent(std::make_unique<agent_t>(agentId, std::forward<TArgs>(args)...));
    }
  }

  template <typename agent_t>
  template <typename... Tn>
    requires(is_agent_v<Tn> && ...)
  void Dynamics<agent_t>::addAgents(Tn... agents) {}

  template <typename agent_t>
  template <typename T1, typename... Tn>
    requires(is_agent_v<T1> && (is_agent_v<Tn> && ...))
  void Dynamics<agent_t>::addAgents(T1 agent, Tn... agents) {
    addAgent(std::make_unique<agent_t>(agent));
    addAgents(agents...);
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addAgents(std::span<agent_t> agents) {
    std::ranges::for_each(agents, [this](const auto& agent) -> void {
      addAgent(std::make_unique<agent_t>(agent));
    });
  }

  template <typename agent_t>
  void Dynamics<agent_t>::removeAgent(Size agentId) {
    m_agents.erase(agentId);
    Logger::debug(std::format("Removed agent with id {}", agentId));
  }

  template <typename agent_t>
  template <typename T1, typename... Tn>
    requires(std::is_convertible_v<T1, Size> && (std::is_convertible_v<Tn, Size> && ...))
  void Dynamics<agent_t>::removeAgents(T1 id, Tn... ids) {
    removeAgent(id);
    removeAgents(ids...);
  }

  template <typename agent_t>
  template <typename... TArgs>
    requires(std::is_constructible_v<Itinerary, TArgs...>)
  void Dynamics<agent_t>::addItinerary(TArgs&&... args) {
    addItinerary(std::make_unique<Itinerary>(std::forward<TArgs>(args)...));
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addItinerary(std::unique_ptr<Itinerary> itinerary) {
    if (m_itineraries.contains(itinerary->id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Itinerary with id {} already exists.", itinerary->id())));
    }
    if (!m_graph.nodes().contains(itinerary->destination())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(std::format(
          "Destination node with id {} not found", itinerary->destination())));
    }
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  template <typename agent_t>
  void Dynamics<agent_t>::resetTime() {
    m_time = 0;
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::agentMeanSpeed() const {
    std::vector<double> speeds;
    if (!m_agents.empty()) {
      speeds.reserve(m_agents.size());
      for (const auto& [agentId, agent] : m_agents) {
        speeds.push_back(agent->speed());
      }
    }
    return Measurement<double>(speeds);
  }

  template <typename agent_t>
  double Dynamics<agent_t>::streetMeanSpeed(Id streetId) const {
    auto const& pStreet{m_graph.edge(streetId)};
    auto const nAgents{pStreet->nAgents()};
    if (nAgents == 0) {
      return 0.;
    }
    double speed{0.};
    for (auto const& agentId : pStreet->movingAgents()) {
      speed += m_agents.at(agentId)->speed();
    }
    return speed / nAgents;
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanSpeed() const {
    std::vector<double> speeds;
    speeds.reserve(m_graph.edges().size());
    for (const auto& [streetId, street] : m_graph.edges()) {
      speeds.push_back(streetMeanSpeed(streetId));
    }
    return Measurement<double>(speeds);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanSpeed(double threshold,
                                                         bool above) const {
    std::vector<double> speeds;
    speeds.reserve(m_graph.nEdges());
    for (const auto& [streetId, street] : m_graph.edges()) {
      if (above && (street->density(true) > threshold)) {
        speeds.push_back(streetMeanSpeed(streetId));
      } else if (!above && (street->density(true) < threshold)) {
        speeds.push_back(streetMeanSpeed(streetId));
      }
    }
    return Measurement<double>(speeds);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanDensity(bool normalized) const {
    if (m_graph.edges().empty()) {
      return Measurement(0., 0.);
    }
    std::vector<double> densities;
    densities.reserve(m_graph.edges().size());
    if (normalized) {
      for (const auto& [streetId, street] : m_graph.edges()) {
        densities.push_back(street->density(true));
      }
    } else {
      double sum{0.};
      for (const auto& [streetId, street] : m_graph.edges()) {
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

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanFlow() const {
    std::vector<double> flows;
    flows.reserve(m_graph.edges().size());
    for (const auto& [streetId, street] : m_graph.edges()) {
      flows.push_back(street->density() * this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanFlow(double threshold,
                                                        bool above) const {
    std::vector<double> flows;
    flows.reserve(m_graph.edges().size());
    for (const auto& [streetId, street] : m_graph.edges()) {
      if (above && (street->density(true) > threshold)) {
        flows.push_back(street->density() * this->streetMeanSpeed(streetId));
      } else if (!above && (street->density(true) < threshold)) {
        flows.push_back(street->density() * this->streetMeanSpeed(streetId));
      }
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::meanSpireInputFlow(bool resetValue) {
    auto deltaTime{m_time - m_previousSpireTime};
    if (deltaTime == 0) {
      return Measurement(0., 0.);
    }
    m_previousSpireTime = m_time;
    std::vector<double> flows;
    flows.reserve(m_graph.nEdges());
    for (const auto& [streetId, street] : m_graph.edges()) {
      if (street->isSpire()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        flows.push_back(static_cast<double>(spire.inputCounts(resetValue)) / deltaTime);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::meanSpireOutputFlow(bool resetValue) {
    auto deltaTime{m_time - m_previousSpireTime};
    if (deltaTime == 0) {
      return Measurement(0., 0.);
    }
    m_previousSpireTime = m_time;
    std::vector<double> flows;
    flows.reserve(m_graph.edges().size());
    for (auto const& [streetId, street] : m_graph.edges()) {
      if (street->isSpire()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        flows.push_back(static_cast<double>(spire.outputCounts(resetValue)) / deltaTime);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  void Dynamics<agent_t>::saveStreetDensities(const std::string& filename,
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
      for (auto const& [streetId, _] : this->m_graph.edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.edges()) {
      // keep 2 decimal digits;
      file << separator << std::fixed << std::setprecision(2)
           << pStreet->density(normalized);
    }
    file << std::endl;
    file.close();
  }
  template <typename agent_t>
  void Dynamics<agent_t>::saveInputStreetCounts(const std::string& filename,
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
      for (auto const& [streetId, _] : this->m_graph.edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.edges()) {
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
  template <typename agent_t>
  void Dynamics<agent_t>::saveOutputStreetCounts(const std::string& filename,
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
      for (auto const& [streetId, _] : this->m_graph.edges()) {
        file << separator << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.edges()) {
      int value{0};
      if (pStreet->isSpire()) {
        if (pStreet->isStochastic()) {
          value = dynamic_cast<StochasticSpireStreet&>(*pStreet).outputCounts(reset);
        } else {
          value = dynamic_cast<SpireStreet&>(*pStreet).outputCounts(reset);
        }
      }
      file << separator << value;
    }
    file << std::endl;
    file.close();
  }
};  // namespace dsm
