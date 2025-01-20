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
    /// @brief Construct a new Street object starting from an existing street
    /// @details The new street has different id but same capacity, length, speed limit, and node pair as the
    ///          existing street.
    /// @param Road The existing street
    /// @param id The new street's id
    Road(Id id, const Road&);
    /// @brief Construct a new Street object
    /// @param id The street's id
    /// @param nodePair The street's node pair
    /// @param length The street's length, in meters (default is the mean vehicle length)
    /// @param nLanes The street's number of lanes (default is 1)
    /// @param maxSpeed The street's speed limit, in m/s (default is 50 km/h)
    /// @param name The street's name (default is an empty string)
    /// @param capacity The street's capacity (default is the maximum number of vehicles that can fit in the street)
    /// @param transportCapacity The street's transport capacity (default is 1)
    Road(Id id,
         std::pair<Id, Id> nodePair,
         double length = m_meanVehicleLength,
         double maxSpeed = 13.8888888889,
         int nLanes = 1,
         std::string name = std::string(),
         std::optional<int> capacity = std::nullopt,
         int transportCapacity = 1);

    static void setMeanVehicleLength(double meanVehicleLength);
    static double meanVehicleLength();

    void setMaxSpeed(double speed);

    double length() const;
    double maxSpeed() const;
    int nLanes() const;
    std::string name() const;

    virtual int nAgents() const = 0;
  };
}  // namespace dsm