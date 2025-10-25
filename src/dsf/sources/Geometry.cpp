#include "../headers/Geometry.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace dsf {
  namespace geometry {
    Point::Point(const std::string& strPoint, const std::string& format) {
      if (format == "WKT") {
        auto start = strPoint.find('(');
        auto end = strPoint.find(')');
        if (start == std::string::npos || end == std::string::npos || end <= start) {
          throw std::invalid_argument("Invalid WKT POINT format: " + strPoint);
        }
        std::string coordStr = strPoint.substr(start + 1, end - start - 1);
        std::istringstream coordStream(coordStr);
        coordStream >> m_x >> m_y;
      } else {
        throw std::invalid_argument("Unsupported format: " + format);
      }
    }

    PolyLine::PolyLine(const std::string& strLine, const std::string& format) {
      if (format == "WKT") {
        auto start = strLine.find('(');
        auto end = strLine.find(')');
        if (start == std::string::npos || end == std::string::npos || end <= start) {
          throw std::invalid_argument("Invalid WKT LINESTRING format: " + strLine);
        }
        std::string coordsStr = strLine.substr(start + 1, end - start - 1);
        std::istringstream coordsStream(coordsStr);
        // Count the number of ',' to estimate points
        std::size_t nPoints = std::count(coordsStr.begin(), coordsStr.end(), ',') + 1;
        this->reserve(nPoints);
        std::string pointStr;
        while (std::getline(coordsStream, pointStr, ',')) {
          std::istringstream pointStream(pointStr);
          double x, y;
          pointStream >> x >> y;
          this->push_back(Point{x, y});
        }
      } else {
        throw std::invalid_argument("Unsupported PolyLine format: " + format);
      }
    }
  }  // namespace geometry
}  // namespace dsf