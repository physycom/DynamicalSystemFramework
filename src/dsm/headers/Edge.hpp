#pragma once

#include <utility>
#include <vector>
#include "../utility/Typedef.hpp"

namespace dsm {
  class Edge {
  protected:
    std::vector<std::pair<double, double>> m_geometry;
    Id m_id;
    std::pair<Id, Id> m_nodePair;
    int m_capacity;
    int m_transportCapacity;
    double m_angle;

    void m_setAngle(std::pair<double, double> srcNodeCoordinates,
                    std::pair<double, double> dstNodeCoordinates);

  public:
    /// @brief Construct a new Edge object
    /// @param id The edge's id
    /// @param nodePair The edge's node pair (u, v) with the edge u -> v
    /// @param capacity The edge's capacity, in number of agents, i.e. the maximum number of agents that can be on the
    /// edge at the same time. Default is 1.
    /// @param transportCapacity The edge's transport capacity, in number of agents, i.e. the maximum number of agents
    /// that can be emitted by the same time. Default is 1.
    /// @param angle The edge's angle, in radians, between the source and destination nodes
    Edge(Id id,
         std::pair<Id, Id> nodePair,
         int capacity = 1,
         int transportCapacity = 1,
         std::vector<std::pair<double, double>> geometry = {});

    void setCapacity(int capacity);
    void setTransportCapacity(int capacity);
    void setGeometry(std::vector<std::pair<double, double>> geometry);

    /// @brief Get the edge's id
    /// @return Id The edge's id
    Id id() const;
    /// @brief Get the edge's source node id
    /// @return Id The edge's source node id
    Id source() const;
    /// @brief Get the edge's target node id
    /// @return Id The edge's target node id
    Id target() const;
    /// @brief Get the edge's node pair
    /// @return std::pair<Id, Id> The edge's node pair, where the first element is the source node id and the second
    /// element is the target node id. The pair is (u, v) with the edge u -> v.
    std::pair<Id, Id> const& nodePair() const;

    std::vector<std::pair<double, double>> const& geometry() const;
    /// @brief Get the edge's capacity, in number of agents
    /// @return int The edge's capacity, in number of agents
    int capacity() const;
    /// @brief Get the edge's transport capacity, in number of agents
    /// @return int The edge's transport capacity, in number of agents
    int transportCapacity() const;
    /// @brief Get the edge's angle, in radians, between the source and target nodes
    /// @return double The edge's angle, in radians
    double angle() const;

    virtual bool isFull() const = 0;

    double deltaAngle(double const previousEdgeAngle) const;
  };
};  // namespace dsm