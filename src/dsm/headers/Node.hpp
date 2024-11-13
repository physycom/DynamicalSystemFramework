/// @file       /src/dsm/headers/Node.hpp
/// @brief      Defines the Node class.
///
/// @details    This file contains the definition of the Node class.
///             The Node class represents a node in the network. It is templated by the type
///             of the node's id, which must be an unsigned integral type.
///             The derived classes are:
///             - Intersection: represents an intersection node with a map of agents
///               - TrafficLight: represents a traffic light intersection node
///             - Roundabout: represents a roundabout node with a queue of agents

#pragma once

#include <functional>
#include <utility>
#include <stdexcept>
#include <optional>
#include <set>
#include <map>
#include <format>
#include <cassert>

#include "../utility/Logger.hpp"
#include "../utility/queue.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The Node class represents the concept of a node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Node {
  protected:
    Id m_id;
    std::optional<std::pair<double, double>> m_coords;
    Size m_capacity;
    Size m_transportCapacity;

  public:
    Node() = default;
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    explicit Node(Id id) : m_id{id}, m_capacity{1}, m_transportCapacity{1} {}
    /// @brief Construct a new Node object with capacity 1
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    Node(Id id, std::pair<double, double> coords)
        : m_id{id}, m_coords{std::move(coords)}, m_capacity{1}, m_transportCapacity{1} {}
    virtual ~Node() = default;

    /// @brief Set the node's id
    /// @param id The node's id
    void setId(Id id) { m_id = id; }
    /// @brief Set the node's coordinates
    /// @param coords A std::pair containing the node's coordinates (lat, lon)
    void setCoords(std::pair<double, double> coords) { m_coords = std::move(coords); }
    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    virtual void setCapacity(Size capacity) { m_capacity = capacity; }
    /// @brief Set the node's transport capacity
    /// @param capacity The node's transport capacity
    virtual void setTransportCapacity(Size capacity) { m_transportCapacity = capacity; }
    /// @brief Get the node's id
    /// @return Id The node's id
    Id id() const { return m_id; }
    /// @brief Get the node's coordinates
    /// @return std::optional<std::pair<double, double>> A std::pair containing the node's coordinates
    const std::optional<std::pair<double, double>>& coords() const { return m_coords; }
    /// @brief Get the node's capacity
    /// @return Size The node's capacity
    Size capacity() const { return m_capacity; }
    /// @brief Get the node's transport capacity
    /// @return Size The node's transport capacity
    Size transportCapacity() const { return m_transportCapacity; }

    virtual bool isFull() const = 0;

    virtual bool isIntersection() const noexcept { return false; }
    virtual bool isTrafficLight() const noexcept { return false; }
    virtual bool isRoundabout() const noexcept { return false; }
  };

  /// @brief The Intersection class represents a node in the network.
  /// @tparam Id The type of the node's id. It must be an unsigned integral type.
  class Intersection : public Node {
  protected:
    std::multimap<int16_t, Id> m_agents;
    std::set<Id>
        m_streetPriorities;  // A set containing the street ids that have priority - like main roads
    Size m_agentCounter;

  public:
    Intersection() = default;
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    explicit Intersection(Id id) : Node{id} {};
    /// @brief Construct a new Intersection object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    Intersection(Id id, std::pair<double, double> coords) : Node{id, coords} {};

    virtual ~Intersection() = default;

    /// @brief Set the node's capacity
    /// @param capacity The node's capacity
    /// @throws std::runtime_error if the capacity is smaller than the current queue size
    void setCapacity(Size capacity) override;

    /// @brief Put an agent in the node
    /// @param agent A std::pair containing the agent's angle difference and id
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node.
    /// @throws std::runtime_error if the node is full
    void addAgent(double angle, Id agentId);
    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @details The agent's angle difference is used to order the agents in the node.
    ///          The agent with the smallest angle difference is the first one to be
    ///          removed from the node.
    /// @throws std::runtime_error if the node is full
    void addAgent(Id agentId);
    /// @brief Removes an agent from the node
    /// @param agentId The agent's id
    void removeAgent(Id agentId);
    /// @brief Set the node streets with priority
    /// @param streetPriorities A std::set containing the node's street priorities
    void setStreetPriorities(std::set<Id> streetPriorities) {
      m_streetPriorities = std::move(streetPriorities);
    }
    /// @brief Add a street to the node street priorities
    /// @param streetId The street's id
    void addStreetPriority(Id streetId) { m_streetPriorities.emplace(streetId); }

    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    bool isFull() const override { return m_agents.size() == this->m_capacity; }

    /// @brief Get the node's street priorities
    /// @details This function returns a std::set containing the node's street priorities.
    ///        If a street has priority, it means that the agents that are on that street
    ///        have priority over the agents that are on the other streets.
    /// @return std::set<Id> A std::set containing the node's street priorities
    virtual const std::set<Id>& streetPriorities() const { return m_streetPriorities; };
    /// @brief Get the node's agent ids
    /// @return std::set<Id> A std::set containing the node's agent ids
    const std::multimap<int16_t, Id>& agents() { return m_agents; };
    /// @brief Returns the number of agents that have passed through the node
    /// @return Size The number of agents that have passed through the node
    /// @details This function returns the number of agents that have passed through the node
    ///          since the last time this function was called. It also resets the counter.
    Size agentCounter();

    virtual bool isIntersection() const noexcept override final { return true; }
  };

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  class TrafficLight : public Intersection {
  private:
    std::optional<std::pair<double, double>> m_leftTurnRatio;
    std::optional<std::pair<Delay, Delay>> m_delay;
    Delay m_counter;
    Delay m_phase;

  public:
    /// @brief Construct a new TrafficLight object
    /// @param id The node's id
    explicit TrafficLight(Id id) : Intersection{id}, m_counter{0}, m_phase{0} {};
    /// @brief Construct a new TrafficLight object
    /// @param node An Intersection object
    TrafficLight(const Node& node);

    /// @brief Set the node's delay
    /// @details This function is used to set the node's delay.
    ///          If the delay is already set, the function will check the counter:
    ///          - if the counter is more than the sum of the new green and red delays, it
    ///            will be set to the new sum minus one, i.e. one more red cycle.
    ///          - if the counter is less than the old green delay but more than the new green delay,
    ///            it will be set to the new green delay minus the difference between the old and the new delay.
    /// @param delay The node's delay
    void setDelay(Delay delay);
    /// @brief Set the node's delay
    /// @details This function is used to set the node's delay.
    ///          If the delay is already set, the function will check the counter:
    ///          - if the counter is more than the sum of the new green and red delays, it
    ///            will be set to the new sum minus one, i.e. one more red cycle.
    ///          - if the counter is less than the old green delay but more than the new green delay,
    ///            it will be set to the new green delay minus the difference between the old and the new delay.
    /// @param delay The node's delay
    void setDelay(std::pair<Delay, Delay> delay);
    /// @brief Set the node's phase
    /// @param phase The node's phase
    /// @throw std::runtime_error if the delay is not set
    void setPhase(Delay phase);
    /// @brief Set the node's left turn ratio
    /// @param ratio A std::pair containing the left turn ratio
    /// @details ratio.first * greentime is the green time for left turns while ratio.second * redtime is the red time for left turns
    /// This is useful for traffic lights when the input street has many lanes and, for example, one resevred for left turns.
    void setLeftTurnRatio(std::pair<double, double> ratio);
    /// @brief Set the node's left turn ratio
    /// @param first The first component of the left turn ratio
    /// @param second The second component of the left turn ratio
    void setLeftTurnRatio(double const first, double const second) {
      setLeftTurnRatio(std::make_pair(first, second));
    }
    /// @brief Set the node's left turn ratio as std::pair(ratio, ratio)
    /// @param ratio The left turn ratio
    void setLeftTurnRatio(double const ratio) {
      setLeftTurnRatio(std::make_pair(ratio, ratio));
    }
    /// @brief Increase the node's counter
    /// @details This function is used to increase the node's counter
    ///          when the simulation is running. It automatically resets the counter
    ///          when it reaches the double of the delay value.
    /// @throw std::runtime_error if the delay is not set
    void increaseCounter();

    /// @brief  Set the phase of the node after the current red-green cycle has passed
    /// @param phase The new node phase
    void setPhaseAfterCycle(Delay phase);

    /// @brief Get the node's delay
    /// @return std::optional<Delay> The node's delay
    std::optional<std::pair<Delay, Delay>> delay() const { return m_delay; }
    Delay counter() const { return m_counter; }
    /// @brief Get the node's left turn ratio
    /// @return std::optional<std::pair<double, double>> The node's left turn ratio
    std::optional<std::pair<double, double>> leftTurnRatio() const {
      return m_leftTurnRatio;
    }
    /// @brief Returns true if the traffic light is green
    /// @return bool True if the traffic light is green
    bool isGreen() const;
    bool isGreen(Id streetId) const;
    bool isGreen(Id streetId, double angle) const;
    bool isTrafficLight() const noexcept override { return true; }
  };

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  TrafficLight<Delay>::TrafficLight(const Node& node)
      : Intersection{node.id()}, m_counter{0}, m_phase{0} {
    if (node.coords().has_value()) {
      this->setCoords(node.coords().value());
    }
    this->setCapacity(node.capacity());
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::setDelay(Delay delay) {
    if (m_delay.has_value()) {
      if (m_counter >= delay + m_delay.value().second) {
        m_counter = delay + m_delay.value().second - 1;
      } else if (delay < m_delay.value().first) {
        if (m_counter >= delay && m_counter <= m_delay.value().first) {
          m_counter = delay - (m_delay.value().first - m_counter);
        }
      }
    }
    m_delay = std::make_pair(delay, delay);
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::setDelay(std::pair<Delay, Delay> delay) {
    if (m_delay.has_value()) {
      if (m_counter >= delay.first + delay.second) {
        m_counter = delay.first + delay.second - 1;
      } else if (delay.first < m_delay.value().first) {
        if (m_counter >= delay.first && m_counter <= m_delay.value().first) {
          m_counter = delay.first - (m_delay.value().first - m_counter);
        }
      }
    }
    m_delay = std::move(delay);
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::setPhase(Delay phase) {
    if (!m_delay.has_value()) {
      throw std::runtime_error(buildLog("TrafficLight's delay has not been set."));
    }
    if (phase > m_delay.value().first + m_delay.value().second) {
      phase -= m_delay.value().first + m_delay.value().second;
    }
    m_counter = phase;
    m_phase = 0;
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::setPhaseAfterCycle(Delay phase) {
    if (phase > m_delay.value().first + m_delay.value().second) {
      phase -= m_delay.value().first + m_delay.value().second;
    }
    m_phase = phase;
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::setLeftTurnRatio(std::pair<double, double> ratio) {
    assert((void("Left turn ratio components must be between 0 and 1."),
            ratio.first >= 0. && ratio.first <= 1. && ratio.second >= 0. &&
                ratio.second <= 1.));
    m_leftTurnRatio = std::move(ratio);
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  void TrafficLight<Delay>::increaseCounter() {
    if (!m_delay.has_value()) {
      throw std::runtime_error(buildLog("TrafficLight's delay has not been set."));
    }
    ++m_counter;
    if (m_counter == m_delay.value().first + m_delay.value().second) {
      if (m_phase != 0) {
        m_counter = m_phase;
        m_phase = 0;
      } else {
        m_counter = 0;
      }
    }
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  bool TrafficLight<Delay>::isGreen() const {
    if (!m_delay.has_value()) {
      throw std::runtime_error(buildLog("TrafficLight's delay has not been set."));
    }
    return m_counter < m_delay.value().first;
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  bool TrafficLight<Delay>::isGreen(Id streetId) const {
    if (!m_delay.has_value()) {
      throw std::runtime_error(buildLog("TrafficLight's delay has not been set."));
    }
    bool hasPriority{this->streetPriorities().contains(streetId)};
    if (this->isGreen()) {
      return hasPriority;
    }
    return !hasPriority;
  }

  template <typename Delay>
    requires(std::unsigned_integral<Delay>)
  bool TrafficLight<Delay>::isGreen(Id streetId, double angle) const {
    assert((void("TrafficLight's delay has not been set."), m_delay.has_value()));
    assert((void("TrafficLight's left turn ratio has not been set."),
            m_leftTurnRatio.has_value()));
    bool const hasPriority{this->streetPriorities().contains(streetId)};
    auto const pair{m_delay.value()};
    if (angle > 0.) {
      if (hasPriority) {
        return m_counter > pair.first * (1. - m_leftTurnRatio.value().first) &&
               m_counter < pair.first;
      } else {
        return m_counter >
               pair.first + pair.second * (1. - m_leftTurnRatio.value().second);
      }
    } else {
      if (hasPriority) {
        return m_counter < pair.first * (1. - m_leftTurnRatio.value().first);
      } else {
        return m_counter > pair.first &&
               m_counter <
                   pair.first + pair.second * (1. - m_leftTurnRatio.value().second);
      }
    }
  }

  /// @brief The Roundabout class represents a roundabout node in the network.
  /// @tparam Id The type of the node's id
  /// @tparam Size The type of the node's capacity
  class Roundabout : public Node {
  protected:
    dsm::queue<Id> m_agents;

  public:
    Roundabout() = default;
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    explicit Roundabout(Id id) : Node{id} {};
    /// @brief Construct a new Roundabout object
    /// @param id The node's id
    /// @param coords A std::pair containing the node's coordinates
    Roundabout(Id id, std::pair<double, double> coords) : Node{id, coords} {};
    /// @brief Construct a new Roundabout object
    /// @param node An Intersection object
    Roundabout(const Node& node);

    virtual ~Roundabout() = default;

    /// @brief Put an agent in the node
    /// @param agentId The agent's id
    /// @throws std::runtime_error if the node is full
    void enqueue(Id agentId);
    /// @brief Removes the first agent from the node
    /// @return Id The agent's id
    Id dequeue();
    /// @brief Get the node's queue
    /// @return dsm::queue<Id> The node's queue
    const dsm::queue<Id>& agents() const { return m_agents; }

    /// @brief Returns true if the node is full
    /// @return bool True if the node is full
    bool isFull() const override { return m_agents.size() == this->m_capacity; }
    /// @brief Returns true if the node is a roundabout
    /// @return bool True if the node is a roundabout
    bool isRoundabout() const noexcept override { return true; }
  };

};  // namespace dsm
