/// @file      /src/dsm/headers/Station.hpp
/// @brief     Defines the Station class.
///
/// @details   The Station class represents a train station in the network.
///            It is a derived class of the Node class.

#pragma once

#include "Node.hpp"

#include <map>

namespace dsm {
  class Station : public Node {
  private:
    Delay m_managementTime;
    Time m_lastEventTime;
    std::multimap<train_t, Id, std::greater<train_t>> m_trains;

  public:
    /// @brief Construct a new Station object
    /// @param id The station's id
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(Id id, Delay managementTime);
    /// @brief Construct a new Station object
    /// @param id The station's id
    /// @param coords A std::pair containing the station's coordinates (lat, lon)
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(Id id, std::pair<double, double> coords, Delay managementTime);
    /// @brief Construct a new Station object
    /// @param node A Node object representing the station
    /// @param managementTime The time it takes between two train departures/arrivals
    Station(Node const& node, Delay managementTime);
    /// @brief Construct a new Station object by copying another Station object
    /// @param other The Station object to copy
    Station(Station const& other);
    /// @brief Enqueue a train in the station
    /// @param trainId The id of the train to enqueue
    /// @param trainType The type of the train to enqueue
    void enqueue(Id trainId, train_t trainType);
    /// @brief Dequeue a train from the station
    /// @return The id of the dequeued train
    std::optional<Id> dequeue();
    /// @brief Get the time it takes between two train departures/arrivals
    /// @return The management time
    Delay managementTime() const;
    /// @brief Get the train density of the station
    /// @return The train density of the station
    double density() const final;
    /// @brief Check if the station is full
    /// @return True if the station is full, false otherwise
    bool isFull() const final;
    /// @brief Check if the station is active
    /// @param currentTime The current time
    /// @return True if the station is active, false otherwise
    /// @details A station is considered active if it has passed management time since the last event
    bool isActive(Time currentTime) const;
    /// @brief Check if the node is a station
    /// @return True
    bool isStation() const noexcept final;
  };
}  // namespace dsm