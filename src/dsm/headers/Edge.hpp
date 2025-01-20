#pragma once

#include <utility>
#include "../utility/Typedef.hpp"

namespace dsm {
  class Edge {
  protected:
    Id m_id;
    std::pair<Id, Id> m_nodePair;
    int m_capacity;
    int m_transportCapacity;
    double m_angle;

  public:
    Edge(Id id,
         std::pair<Id, Id> nodePair,
         int capacity = 1,
         int transportCapacity = 1,
         double angle = 0.0);

    void setCapacity(int capacity);
    void setTransportCapacity(int capacity);
    void setAngle(std::pair<double, double> srcNodeCoordinates,
                  std::pair<double, double> dstNodeCoordinates);

    Id id() const;
    Id u() const;
    Id v() const;
    std::pair<Id, Id> nodePair() const;
    int capacity() const;
    int transportCapacity() const;
    double angle() const;

    virtual bool isFull() const = 0;
  };
};  // namespace dsm