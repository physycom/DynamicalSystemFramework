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

#include "DijkstraWeights.hpp"
#include "Itinerary.hpp"
#include "Graph.hpp"
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
    bool m_bCacheEnabled;

  protected:
    Graph m_graph;
    Time m_time, m_previousSpireTime;
    std::mt19937_64 m_generator;

    virtual void m_evolveStreet(const std::unique_ptr<Street>& pStreet,
                                bool reinsert_agents) = 0;
    virtual bool m_evolveNode(const std::unique_ptr<Node>& pNode) = 0;
    virtual void m_evolveAgents() = 0;

    /// @brief Update the path of a single itinerary using Dijsktra's algorithm
    /// @param pItinerary An std::unique_prt to the itinerary
    void m_updatePath(const std::unique_ptr<Itinerary>& pItinerary) {
      if (m_bCacheEnabled) {
        auto const& file =
            std::format("{}it{}.dsmcache", g_cacheFolder, pItinerary->id());
        if (std::filesystem::exists(file)) {
          auto path = SparseMatrix<bool>{};
          path.load(file);
          pItinerary->setPath(std::move(path));
          Logger::info(std::format("Loaded cached path for itinerary {}", pItinerary->id()));
          return;
        }
      }
      Size const dimension = m_graph.adjMatrix().getRowDim();
      auto const destinationID = pItinerary->destination();
      SparseMatrix<bool> path{dimension, dimension};

      // Cache shortest paths to avoid redundant calculations
      std::unordered_map<Id, std::optional<DijkstraResult>> shortestPaths;

      for (const auto& [nodeId, node] : m_graph.nodeSet()) {
        if (nodeId == destinationID || m_graph.adjMatrix().getRow(nodeId).empty()) {
          continue;
        }

        // Compute and cache shortest path from nodeId to destination
        auto result = m_graph.shortestPath(nodeId, destinationID);
        if (!result.has_value()) {
          Logger::warning(
              std::format("No path found from node {} to {}", nodeId, destinationID));
          continue;
        }

        const auto minDistance = result.value().distance();
        shortestPaths[nodeId] = result;  // Cache for reuse

        for (const auto& [nextNodeId, _] : m_graph.adjMatrix().getRow(nodeId)) {
          if (nextNodeId == destinationID) {
            if (minDistance ==
                m_graph.streetSet()[nodeId * dimension + nextNodeId]->length()) {
              path.insert(nodeId, nextNodeId, true);
            }
            continue;
          }

          // Use cached shortest path if available
          auto it = shortestPaths.find(nextNodeId);
          if (it == shortestPaths.end()) {
            shortestPaths[nextNodeId] = m_graph.shortestPath(nextNodeId, destinationID);
          }

          if (shortestPaths[nextNodeId].has_value()) {
            const auto nextDistance = shortestPaths[nextNodeId].value().distance();
            if (minDistance ==
                nextDistance +
                    m_graph.streetSet()[nodeId * dimension + nextNodeId]->length()) {
              path.insert(nodeId, nextNodeId, true);
            }
          }
        }
      }

      if (path.size() == 0) {
        Logger::error(
            std::format("Path with id {} and destination {} is empty. Please check the "
                        "adjacency matrix.",
                        pItinerary->id(),
                        pItinerary->destination()));
      }

      pItinerary->setPath(path);
      if (m_bCacheEnabled) {
        pItinerary->path().cache(
            std::format("{}it{}.dsmcache", g_cacheFolder, pItinerary->id()));
        Logger::info(std::format("Cached path for itinerary {}", pItinerary->id()));
      }
    }

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    Dynamics(Graph& graph, std::optional<unsigned int> seed);

    virtual void setAgentSpeed(Size agentId) = 0;
    virtual void evolve(bool reinsert_agents = false) = 0;

    /// @brief Update the paths of the itineraries based on the actual travel times
    virtual void updatePaths();

    /// @brief Set the dynamics destination nodes auto-generating itineraries
    /// @param destinationNodes The destination nodes
    /// @param updatePaths If true, the paths are updated
    /// @throws std::invalid_argument Ifone or more destination nodes do not exist
    void setDestinationNodes(const std::span<Id>& destinationNodes,
                             bool updatePaths = true);

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
    /// @param itinerary The itinerary
    void addItinerary(const Itinerary& itinerary);
    /// @brief Add an itinerary
    /// @param itinerary std::unique_ptr to the itinerary
    void addItinerary(std::unique_ptr<Itinerary> itinerary);
    template <typename... Tn>
      requires(is_itinerary_v<Tn> && ...)
    void addItineraries(Tn... itineraries);
    /// @brief Add a pack of itineraries
    /// @tparam T1
    /// @tparam ...Tn
    /// @param itinerary
    /// @param ...itineraries
    template <typename T1, typename... Tn>
      requires(is_itinerary_v<T1> && (is_itinerary_v<Tn> && ...))
    void addItineraries(T1 itinerary, Tn... itineraries);
    /// @brief Add a set of itineraries
    /// @param itineraries Generic container of itineraries, represented by an std::span
    void addItineraries(std::span<Itinerary> itineraries);

    void enableCache();

    /// @brief Reset the simulation time
    void resetTime();

    /// @brief Get the graph
    /// @return const Graph&, The graph
    const Graph& graph() const { return m_graph; };
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
    void saveStreetDensities(const std::string& filename, bool normalized = true) const;
    /// @brief Save the street input counts in csv format
    /// @param filename The name of the file
    /// @param reset If true, the input counts are cleared after the computation
    /// @details NOTE: counts are printed only if the street is a spire
    void saveInputStreetCounts(const std::string& filename, bool reset = false);
    /// @brief Save the street output counts in csv format
    /// @param filename The name of the file
    /// @param reset If true, the output counts are cleared after the computation
    /// @details NOTE: counts are printed only if the street is a spire
    void saveOutputStreetCounts(const std::string& filename, bool reset = false);
  };

  template <typename agent_t>
  Dynamics<agent_t>::Dynamics(Graph& graph, std::optional<unsigned int> seed)
      : m_bCacheEnabled{true},
        m_graph{std::move(graph)},
        m_time{0},
        m_previousSpireTime{0},
        m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(seed.value());
    }
    for (const auto& nodeId : this->m_graph.outputNodes()) {
      addItinerary(Itinerary{nodeId, nodeId});
      // m_updatePath(m_itineraries.at(nodeId));
    }
    updatePaths();
  }

  template <typename agent_t>
  void Dynamics<agent_t>::updatePaths() {
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(g_cacheFolder)) {
        std::filesystem::create_directory(g_cacheFolder);
      }
    }

    std::vector<std::thread> threads;
    threads.reserve(m_itineraries.size());
    std::exception_ptr pThreadException;
    Logger::info(std::format("Init computing {} paths", m_itineraries.size()));
    for (const auto& [itineraryId, itinerary] : m_itineraries) {
      threads.emplace_back(std::thread([this, &itinerary, &pThreadException] {
        try {
          this->m_updatePath(itinerary);
        } catch (...) {
          if (!pThreadException)
            pThreadException = std::current_exception();
        }
      }));
    }
    for (auto& thread : threads) {
      thread.join();
    }
    // Throw the exception launched first
    if (pThreadException)
      std::rethrow_exception(pThreadException);

    Logger::info("End computing paths");
  }

  template <typename agent_t>
  void Dynamics<agent_t>::setDestinationNodes(const std::span<Id>& destinationNodes,
                                              bool updatePaths) {
    for (const auto& nodeId : destinationNodes) {
      if (!m_graph.nodeSet().contains(nodeId)) {
        Logger::error(std::format("Node with id {} not found", nodeId));
      }
      this->addItinerary(Itinerary{nodeId, nodeId});
    }
    if (updatePaths) {
      this->updatePaths();
    }
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addAgent(std::unique_ptr<agent_t> agent) {
    if (m_agents.size() + 1 > m_graph.maxCapacity()) {
      throw std::overflow_error(Logger::buildExceptionMessage(
          std::format("Graph is already holding the max possible number of agents ({})",
                      m_graph.maxCapacity())));
    }
    if (m_agents.contains(agent->id())) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("Agent with id {} already exists.", agent->id())));
    }
    m_agents.emplace(agent->id(), std::move(agent));
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
  }

  template <typename agent_t>
  template <typename T1, typename... Tn>
    requires(std::is_convertible_v<T1, Size> && (std::is_convertible_v<Tn, Size> && ...))
  void Dynamics<agent_t>::removeAgents(T1 id, Tn... ids) {
    removeAgent(id);
    removeAgents(ids...);
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addItinerary(const Itinerary& itinerary) {
    m_itineraries.emplace(itinerary.id(), std::make_unique<Itinerary>(itinerary));
  }

  template <typename agent_t>
  void Dynamics<agent_t>::addItinerary(std::unique_ptr<Itinerary> itinerary) {
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  template <typename agent_t>
  template <typename... Tn>
    requires(is_itinerary_v<Tn> && ...)
  void Dynamics<agent_t>::addItineraries(Tn... itineraries) {
    (this->addItinerary(itineraries), ...);
  }
  template <typename agent_t>
  void Dynamics<agent_t>::addItineraries(std::span<Itinerary> itineraries) {
    std::ranges::for_each(itineraries, [this](const auto& itinerary) -> void {
      m_itineraries.insert(std::make_unique<Itinerary>(itinerary));
    });
  }

  template <typename agent_t>
  void Dynamics<agent_t>::enableCache() {
    m_bCacheEnabled = true;
    Logger::info(std::format("Cache enabled (default folder is {})", g_cacheFolder));
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
    auto const& pStreet{m_graph.street(streetId)};
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
    speeds.reserve(m_graph.streetSet().size());
    for (const auto& [streetId, street] : m_graph.streetSet()) {
      speeds.push_back(streetMeanSpeed(streetId));
    }
    return Measurement<double>(speeds);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanSpeed(double threshold,
                                                         bool above) const {
    std::vector<double> speeds;
    speeds.reserve(m_graph.streetSet().size());
    for (const auto& [streetId, street] : m_graph.streetSet()) {
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
    if (m_graph.streetSet().size() == 0) {
      return Measurement(0., 0.);
    }
    std::vector<double> densities;
    densities.reserve(m_graph.streetSet().size());
    if (normalized) {
      for (const auto& [streetId, street] : m_graph.streetSet()) {
        densities.push_back(street->density(true));
      }
    } else {
      double sum{0.};
      for (const auto& [streetId, street] : m_graph.streetSet()) {
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
    flows.reserve(m_graph.streetSet().size());
    for (const auto& [streetId, street] : m_graph.streetSet()) {
      flows.push_back(street->density() * this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  Measurement<double> Dynamics<agent_t>::streetMeanFlow(double threshold,
                                                        bool above) const {
    std::vector<double> flows;
    flows.reserve(m_graph.streetSet().size());
    for (const auto& [streetId, street] : m_graph.streetSet()) {
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
    flows.reserve(m_graph.streetSet().size());
    for (const auto& [streetId, street] : m_graph.streetSet()) {
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
    flows.reserve(m_graph.streetSet().size());
    for (auto const& [streetId, street] : m_graph.streetSet()) {
      if (street->isSpire()) {
        auto& spire = dynamic_cast<SpireStreet&>(*street);
        flows.push_back(static_cast<double>(spire.outputCounts(resetValue)) / deltaTime);
      }
    }
    return Measurement<double>(flows);
  }

  template <typename agent_t>
  void Dynamics<agent_t>::saveStreetDensities(const std::string& filename,
                                              bool normalized) const {
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
      for (auto const& [streetId, _] : this->m_graph.streetSet()) {
        file << ';' << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.streetSet()) {
      // keep 2 decimal digits;
      file << ';' << std::fixed << std::setprecision(2) << pStreet->density(normalized);
    }
    file << std::endl;
    file.close();
  }
  template <typename agent_t>
  void Dynamics<agent_t>::saveInputStreetCounts(const std::string& filename, bool reset) {
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
      for (auto const& [streetId, _] : this->m_graph.streetSet()) {
        file << ';' << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.streetSet()) {
      int value{0};
      if (pStreet->isSpire()) {
        if (pStreet->isStochastic()) {
          value = dynamic_cast<StochasticSpireStreet&>(*pStreet).inputCounts(reset);
        } else {
          value = dynamic_cast<SpireStreet&>(*pStreet).inputCounts(reset);
        }
      }
      file << ';' << value;
    }
    file << std::endl;
    file.close();
  }
  template <typename agent_t>
  void Dynamics<agent_t>::saveOutputStreetCounts(const std::string& filename,
                                                 bool reset) {
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
      for (auto const& [streetId, _] : this->m_graph.streetSet()) {
        file << ';' << streetId;
      }
      file << std::endl;
    }
    file << this->time();
    for (auto const& [_, pStreet] : this->m_graph.streetSet()) {
      int value{0};
      if (pStreet->isSpire()) {
        if (pStreet->isStochastic()) {
          value = dynamic_cast<StochasticSpireStreet&>(*pStreet).outputCounts(reset);
        } else {
          value = dynamic_cast<SpireStreet&>(*pStreet).outputCounts(reset);
        }
      }
      file << ';' << value;
    }
    file << std::endl;
    file.close();
  }
};  // namespace dsm
