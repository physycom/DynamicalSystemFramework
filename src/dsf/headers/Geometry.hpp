#pragma once

#include <cmath>
#include <format>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "fmt/format.h"

namespace dsf {
  /// @brief Geometry related functions
  namespace geometry {
    struct Point {
      double x;
      double y;

      bool operator==(const Point& other) const {
        return std::abs(x - other.x) < std::numeric_limits<double>::epsilon() &&
               std::abs(y - other.y) < std::numeric_limits<double>::epsilon();
      }
    };

    /// @brief Parse a WKT POINT string and return a Point object. A WKT POINT is in the format "POINT(x y)" or "POINT (lon lat)".
    /// @param strWKTPoint The WKT POINT string to parse
    /// @return A Point object representing the coordinates of the point
    /// @throws std::invalid_argument if the input string is not a valid WKT POINT
    Point parseWKTPoint(const std::string& strWKTPoint);

    /// @brief A polyline represented as a vector of Points
    struct PolyLine : public std::vector<Point> {
      using std::vector<Point>::vector;  // Inherit constructors
    };

    /// @brief Parse a WKT LINESTRING string and return a PolyLine object. A WKT LINESTRING is in the format "LINESTRING(x1 y1, x2 y2, ...)".
    /// @param strWKTLineString The WKT LINESTRING string to parse
    /// @return A PolyLine object representing the coordinates of the linestring
    /// @throws std::invalid_argument if the input string is not a valid WKT LINESTRING
    PolyLine parseWKTLineString(const std::string& strWKTLineString);
  }  // namespace geometry
}  // namespace dsf

// Specialization of std::formatter for dsf::geometry::Point
template <>
struct std::formatter<dsf::geometry::Point> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::geometry::Point const& point, FormatContext&& ctx) {
    return std::format_to(ctx.out(), "POINT ({}, {})", point.x, point.y);
  }
};
// Specialization of fmt::formatter for dsf::geometry::Point (for fmt library compatibility)
template <>
struct fmt::formatter<dsf::geometry::Point> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::geometry::Point const& point, FormatContext& ctx) const {
    return fmt::format_to(ctx.out(), "POINT ({}, {})", point.x, point.y);
  }
};
// Specialization of std::formatter for dsf::geometry::PolyLine
template <>
struct std::formatter<dsf::geometry::PolyLine> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(dsf::geometry::PolyLine const& polyline, FormatContext&& ctx) {
    auto out = std::format_to(ctx.out(), "LINESTRING (");
    for (std::size_t i = 0; i < polyline.size(); ++i) {
      if (i > 0) {
        out = std::format_to(out, ", ");
      }
      out = std::format_to(out, "{} {}", polyline[i].x, polyline[i].y);
    }
    return std::format_to(out, ")");
  }
};
// Specialization of fmt::formatter for dsf::geometry::PolyLine (for fmt library compatibility)
template <>
struct fmt::formatter<dsf::geometry::PolyLine> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }
  template <typename FormatContext>
  auto format(dsf::geometry::PolyLine const& polyline, FormatContext& ctx) const {
    auto out = fmt::format_to(ctx.out(), "LINESTRING (");
    for (std::size_t i = 0; i < polyline.size(); ++i) {
      if (i > 0) {
        out = fmt::format_to(out, ", ");
      }
      out = fmt::format_to(out, "{} {}", polyline[i].x, polyline[i].y);
    }
    return fmt::format_to(out, ")");
  }
};