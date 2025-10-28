#include "RoadJunction.hpp"

namespace dsf::mobility {
  RoadJunction::RoadJunction(Id id) : Node(id), m_capacity{1}, m_transportCapacity{1.} {}
  RoadJunction::RoadJunction(Id id, geometry::Point point)
      : Node(id, point), m_capacity{1}, m_transportCapacity{1.} {}
  RoadJunction::RoadJunction(RoadJunction const& other)
      : Node(other),
        m_capacity{other.m_capacity},
        m_transportCapacity{other.m_transportCapacity} {}

  void RoadJunction::setCapacity(Size capacity) { m_capacity = capacity; }
  void RoadJunction::setTransportCapacity(double capacity) {
    assert(capacity > 0.);
    m_transportCapacity = capacity;
  }

  Size RoadJunction::capacity() const { return m_capacity; }
  double RoadJunction::transportCapacity() const { return m_transportCapacity; }
  double RoadJunction::density() const { return 0.; }
  bool RoadJunction::isFull() const { return true; }

  bool RoadJunction::isIntersection() const noexcept { return false; }
  bool RoadJunction::isTrafficLight() const noexcept { return false; }
  bool RoadJunction::isRoundabout() const noexcept { return false; }
}  // namespace dsf::mobility