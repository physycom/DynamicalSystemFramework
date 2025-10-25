#include "../headers/Geometry.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace dsf {
  namespace geometry {
    Point parseWKTPoint(const std::string& strWKTPoint) {
      auto start = strWKTPoint.find('(');
      auto end = strWKTPoint.find(')');
      if (start == std::string::npos || end == std::string::npos || end <= start) {
        throw std::invalid_argument("Invalid WKT POINT format: " + strWKTPoint);
      }
      std::string coordStr = strWKTPoint.substr(start + 1, end - start - 1);
      std::istringstream coordStream(coordStr);
      double x, y;
      coordStream >> x >> y;
      return Point{x, y};
    }

    PolyLine parseWKTLineString(const std::string& strWKTLineString) {
      auto start = strWKTLineString.find('(');
      auto end = strWKTLineString.find(')');
      if (start == std::string::npos || end == std::string::npos || end <= start) {
        throw std::invalid_argument("Invalid WKT LINESTRING format: " + strWKTLineString);
      }
      std::string coordsStr = strWKTLineString.substr(start + 1, end - start - 1);
      std::istringstream coordsStream(coordsStr);
      PolyLine polyline;
      // Count the number of ',' to estimate points
      std::size_t nPoints = std::count(coordsStr.begin(), coordsStr.end(), ',') + 1;
      polyline.reserve(nPoints);
      std::string pointStr;
      while (std::getline(coordsStream, pointStr, ',')) {
        std::istringstream pointStream(pointStr);
        double x, y;
        pointStream >> x >> y;
        polyline.push_back(Point{x, y});
      }
      return polyline;
    }
  }  // namespace geometry
}  // namespace dsf