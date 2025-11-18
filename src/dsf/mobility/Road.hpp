#pragma once

#include "../base/Edge.hpp"

#include <memory>
#include <optional>
#include <set>
#include <string>

namespace dsf::mobility {
  class Road : public Edge {
  protected:
    static double m_meanVehicleLength;
    double m_length;
    double m_maxSpeed;
    int m_nLanes;
    int m_capacity;
    double m_transportCapacity;
    std::string m_name;
    int m_priority;
    std::set<Id> m_forbiddenTurns;  // Stores the forbidden turns (road ids)

  public:
    /// @brief Construct a new Road object
    /// @param id The road's id
    /// @param nodePair The road's node pair
    /// @param length The road's length, in meters (default is the mean vehicle length)
    /// @param nLanes The road's number of lanes (default is 1)
    /// @param maxSpeed The road's speed limit, in m/s (default is 50 km/h)
    /// @param name The road's name (default is an empty string)
    /// @param capacity The road's capacity (default is the maximum number of vehicles that can fit in the road)
    /// @param transportCapacity The road's transport capacity (default is 1)
    Road(Id id,
         std::pair<Id, Id> nodePair,
         double length = m_meanVehicleLength,
         double maxSpeed = 13.8888888889,
         int nLanes = 1,
         std::string name = std::string(),
         geometry::PolyLine geometry = {},
         std::optional<int> capacity = std::nullopt,
         double transportCapacity = 1.);
    /// @brief Set the mean vehicle length, in meters (default is 5)
    /// @param meanVehicleLength The mean vehicle length
    /// @throws std::invalid_argument If the mean vehicle length is less or equal to 0
    static void setMeanVehicleLength(double meanVehicleLength);
    /// @brief Get the mean vehicle length
    /// @return double The mean vehicle length
    static double meanVehicleLength();

    /// @brief Set the maximum speed, in meters per second (default is 50 km/h)
    /// @param speed The maximum speed
    /// @throws std::invalid_argument If the speed is less or equal to 0
    void setMaxSpeed(double speed);
    /// @brief Set the capacity, in number of agents
    /// @param capacity The capacity
    /// @throws std::invalid_argument If the capacity is less or equal to 0
    void setCapacity(int capacity);
    /// @brief Set the transport capacity, in number of agents
    /// @param transportCapacity The transport capacity
    /// @throws std::invalid_argument If the transport capacity is less or equal to 0
    void setTransportCapacity(double transportCapacity);
    /// @brief Set the road's priority
    /// @param priority The road's priority
    void setPriority(int priority);
    /// @brief Add a road id to the forbidden turns
    /// @param roadId The road id to add
    void addForbiddenTurn(Id roadId);
    /// @brief Replace the road's forbidden turns with the given set
    /// @param forbiddenTurns The set of forbidden turns
    void setForbiddenTurns(std::set<Id> const& forbiddenTurns);

    /// @brief Get the length, in meters
    /// @return double The length, in meters
    double length() const;
    /// @brief Get the maximum speed, in meters per second
    /// @return double The maximum speed, in meters per second
    double maxSpeed() const;
    /// @brief Get the number of lanes
    /// @return int The number of lanes
    int nLanes() const;
    /// @brief Get the road's capacity, in number of agents
    /// @return int The road's capacity, in number of agents
    int capacity() const;
    /// @brief Get the road's transport capacity, in number of agents
    /// @return double The road's transport capacity, in number of agents
    double transportCapacity() const;
    /// @brief Get the name
    /// @return std::string The name
    std::string name() const;
    /// @brief Get the priority
    /// @return int The priority
    int priority() const;
    /// @brief Get the road's forbidden turns
    /// @return std::set<Id> The road's forbidden turns
    /// @details The forbidden turns are the road ids that are not allowed to be used by the agents
    ///          when they are on the road.
    std::set<Id> const& forbiddenTurns() const;
    /// @brief Get the road's turn direction given the previous road angle
    /// @param previousStreetAngle The angle of the previous road
    /// @return Direction The turn direction
    /// @details The turn direction is the direction that the agent must take when it is on the road.
    ///          The possible values are:
    ///          - UTURN (abs of delta is greater than pi)
    ///          - STRAIGHT (abs of delta is less than pi /8)
    ///          - RIGHT (delta is negative and not covered by the above conditions)
    ///          - LEFT (delta is positive and not covered by the above conditions)
    Direction turnDirection(double const& previousStreetAngle) const;

    virtual int nAgents() const = 0;
    virtual int nMovingAgents() const = 0;
    virtual double nExitingAgents(Direction direction, bool normalizeOnNLanes) const = 0;
    virtual double density(bool normalized = false) const = 0;
  };
}  // namespace dsf::mobility