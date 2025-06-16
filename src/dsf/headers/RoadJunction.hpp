#pragma once

#include "Node.hpp"

#include "../utility/Typedef.hpp"

namespace dsf {
  class RoadJunction : public Node {
    Size m_capacity;
    double m_transportCapacity;

  public:
    explicit RoadJunction(Id id);
    RoadJunction(Id id, std::pair<double, double> coords);
    RoadJunction(RoadJunction const& other);

    RoadJunction& operator=(RoadJunction const& other) {
      if (this != &other) {
        Node::operator=(other);
        m_capacity = other.m_capacity;
        m_transportCapacity = other.m_transportCapacity;
      }
      return *this;
    }

    /// @brief Set the junction's capacity
    /// @param capacity The junction's capacity
    virtual void setCapacity(Size capacity);
    /// @brief Set the junction's transport capacity
    /// @param capacity The junction's transport capacity
    void setTransportCapacity(double capacity);

    /// @brief Get the junction's capacity
    /// @return Size The junction's capacity
    Size capacity() const;
    /// @brief Get the junction's transport capacity
    /// @return Size The junction's transport capacity
    double transportCapacity() const;

    virtual double density() const;
    virtual bool isFull() const;

    virtual bool isIntersection() const noexcept;
    virtual bool isTrafficLight() const noexcept;
    virtual bool isRoundabout() const noexcept;
  };
}  // namespace dsf