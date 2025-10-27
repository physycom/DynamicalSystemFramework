#include "Point.hpp"

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
        double x, y;
        if (!(coordStream >> x >> y)) {
          throw std::invalid_argument("Malformed WKT POINT coordinates: " + strPoint);
        }
        m_x = x;
        m_y = y;
      } else {
        throw std::invalid_argument("Unsupported format: " + format);
      }
    }
  }  // namespace geometry
}  // namespace dsf
