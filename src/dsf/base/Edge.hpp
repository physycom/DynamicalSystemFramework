#pragma once

#include "../geometry/Point.hpp"
#include "../geometry/PolyLine.hpp"
#include "../utility/Typedef.hpp"

#include <format>
#include <optional>
#include <utility>
#include <vector>

namespace dsf {
  class Edge {
  protected:
    geometry::PolyLine m_geometry;
    Id m_id;
    std::pair<Id, Id> m_nodePair;
    std::optional<double> m_weight;
    double m_angle;

    void m_setAngle(geometry::Point srcNodeCoordinates,
                    geometry::Point dstNodeCoordinates);

  public:
    /// @brief Construct a new Edge object
    /// @param id The edge's id
    /// @param nodePair The edge's node pair (u, v) with the edge u -> v
    /// @param geometry The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's
    /// geometry. Default is an empty vector.
    Edge(Id id, std::pair<Id, Id> nodePair, geometry::PolyLine geometry = {});
    Edge(Edge&&) = default;
    Edge(const Edge&) = delete;
    virtual ~Edge() = default;

    /// @brief Set the edge's geometry
    /// @param geometry dsf::geometry::PolyLine The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's geometry
    void setGeometry(geometry::PolyLine geometry);
    /// @brief Set the edge's weight
    /// @param weight The edge's weight
    /// @throws std::invalid_argument if the weight is less or equal to 0
    void setWeight(double const weight);

    /// @brief Get the edge's id
    /// @return Id The edge's id
    inline auto id() const { return m_id; }
    /// @brief Get the edge's source node id
    /// @return Id The edge's source node id
    inline auto source() const { return m_nodePair.first; }
    /// @brief Get the edge's target node id
    /// @return Id The edge's target node id
    inline auto target() const { return m_nodePair.second; }
    /// @brief Get the edge's node pair
    /// @return std::pair<Id, Id> The edge's node pair, where the first element is the source node id and the second
    /// element is the target node id. The pair is (u, v) with the edge u -> v.
    inline auto const& nodePair() const { return m_nodePair; }
    /// @brief Get the edge's geometry
    /// @return dsf::geometry::PolyLine The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's geometry
    inline auto const& geometry() const { return m_geometry; }

    /// @brief Get the edge's angle, in radians, between the source and target nodes
    /// @return double The edge's angle, in radians
    inline auto angle() const { return m_angle; }
    /// @brief Get the edge's weight
    /// @return double The edge's weight
    double weight() const;

    virtual bool isFull() const = 0;

    double deltaAngle(double const previousEdgeAngle) const;
  };
};  // namespace dsf

template <>
struct std::formatter<dsf::Edge> {
  constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const dsf::Edge& edge, FormatContext&& ctx) const {
    return std::format_to(ctx.out(),
                          "Edge(id={}, source={}, target={})",
                          edge.id(),
                          edge.source(),
                          edge.target());
  }
};