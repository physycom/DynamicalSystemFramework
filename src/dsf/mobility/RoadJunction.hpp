#pragma once

#include "../base/Node.hpp"

#include "../utility/Typedef.hpp"

#include <format>

namespace dsf::mobility {
  class RoadJunction : public Node {
    Size m_capacity;
    double m_transportCapacity;

  public:
    explicit RoadJunction(Id id);
    RoadJunction(Id id, geometry::Point coords);
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
}  // namespace dsf::mobility

// Specialization of std::formatter for dsf::mobility::RoadJunction
template <>
struct std::formatter<dsf::mobility::RoadJunction> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::mobility::RoadJunction const& junction, FormatContext&& ctx) const {
    auto out =
        std::format_to(ctx.out(),
                       "RoadJunction(id: {}, name: {}, capacity: {}, transportCapacity: "
                       "{}, coords: ",
                       junction.id(),
                       junction.name(),
                       junction.capacity(),
                       junction.transportCapacity());
    if (junction.geometry().has_value()) {
      out = std::format_to(out, "{})", *junction.geometry());
    } else {
      out = std::format_to(out, "N/A)");
    }
    return out;
  }
};