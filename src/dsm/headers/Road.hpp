#pragma once

#include "Edge.hpp"

#include <optional>
#include <string>

namespace dsm {
  class Road : public Edge {
  protected:
    static double m_meanVehicleLength;
    double m_length;
    double m_maxSpeed;
    int m_nLanes;
    std::string m_name;

  public:
    /// @brief Construct a new Road object starting from an existing road
    /// @details The new road has different id but same capacity, length, speed limit, and node pair as the
    ///          existing road.
    /// @param Road The existing road
    /// @param id The new road's id
    Road(Id id, const Road&);
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
         std::optional<int> capacity = std::nullopt,
         int transportCapacity = 1);
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

    /// @brief Get the length, in meters
    /// @return double The length, in meters
    double length() const;
    /// @brief Get the maximum speed, in meters per second
    /// @return double The maximum speed, in meters per second
    double maxSpeed() const;
    /// @brief Get the number of lanes
    /// @return int The number of lanes
    int nLanes() const;
    /// @brief Get the name
    /// @return std::string The name
    std::string name() const;

    virtual void addAgent(Id agentId) = 0;

    virtual int nAgents() const = 0;
    virtual int nMovingAgents() const = 0;
    virtual int nExitingAgents() const = 0;
    virtual double density(bool normalized = false) const = 0;
  };
}  // namespace dsm