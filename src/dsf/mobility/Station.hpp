/// @file      /src/dsf/headers/Station.hpp
/// @brief     Defines the Station class.
///
/// @details   The Station class represents a train station in the network.
///            It is a derived class of the Node class.

#pragma once

#include "RoadJunction.hpp"

#include <map>

namespace dsf::mobility {
  class Station : public RoadJunction {
  private:
    Delay m_managementTime;
    std::multimap<train_t, Id, std::greater<train_t>> m_trains;

  public:
    /// @brief Construct a new Station object
    /// @param id The station's id
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(Id id, Delay managementTime);
    /// @brief Construct a new Station object
    /// @param id The station's id
    /// @param point A geometry::Point containing the station's coordinates
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(Id id, geometry::Point point, Delay managementTime);
    /// @brief Construct a new Station object
    /// @param node A Node object representing the station
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(RoadJunction const& node, Delay managementTime);
    /// @brief Construct a new Station object by copying another Station object
    /// @param other The Station object to copy
    Station(Station const& other);
    /// @brief Enqueue a train in the station
    /// @param trainId The id of the train to enqueue
    /// @param trainType The type of the train to enqueue
    void enqueue(Id trainId, train_t trainType);
    /// @brief Dequeue a train from the station
    /// @return The id of the dequeued train
    Id dequeue();
    /// @brief Get the time it takes between two train departures/arrivals
    /// @return The management time
    Delay managementTime() const;
    /// @brief Get the train density of the station
    /// @return The train density of the station
    double density() const;
    /// @brief Check if the station is full
    /// @return True if the station is full, false otherwise
    bool isFull() const;
    /// @brief Check if the node is a station
    /// @return True
    constexpr bool isStation() const noexcept final { return true; }
  };
}  // namespace dsf::mobility