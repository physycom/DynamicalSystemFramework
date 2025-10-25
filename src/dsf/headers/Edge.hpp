#pragma once

#include <format>
#include <optional>
#include <utility>
#include <vector>
#include "../utility/Typedef.hpp"

namespace dsf {
  class Edge {
  protected:
    std::vector<std::pair<double, double>> m_geometry;
    Id m_id;
    std::pair<Id, Id> m_nodePair;
    std::optional<double> m_weight;
    double m_angle;

    void m_setAngle(std::pair<double, double> srcNodeCoordinates,
                    std::pair<double, double> dstNodeCoordinates);

  public:
    /// @brief Construct a new Edge object
    /// @param id The edge's id
    /// @param nodePair The edge's node pair (u, v) with the edge u -> v
    /// @param geometry The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's
    /// geometry. Default is an empty vector.
    Edge(Id id,
         std::pair<Id, Id> nodePair,
         std::vector<std::pair<double, double>> geometry = {});
    Edge(Edge&&) = default;
    Edge(const Edge&) = delete;
    virtual ~Edge() = default;

    void resetId(Id newId);
    void setGeometry(std::vector<std::pair<double, double>> geometry);
    void setWeight(double const weight);

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
    /// @brief Get the edge's geometry
    /// @return std::vector<std::pair<double, double>> The edge's geometry, a vector of pairs of doubles representing the coordinates of the edge's geometry
    std::vector<std::pair<double, double>> const& geometry() const;

    /// @brief Get the edge's angle, in radians, between the source and target nodes
    /// @return double The edge's angle, in radians
    double angle() const;
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