#ifndef dsf_hpp
#define dsf_hpp

#include <cstdint>
#include <format>

static constexpr uint8_t DSF_VERSION_MAJOR = 3;
static constexpr uint8_t DSF_VERSION_MINOR = 13;
static constexpr uint8_t DSF_VERSION_PATCH = 5;

static auto const DSF_VERSION =
    std::format("{}.{}.{}", DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH);

namespace dsf {
  /// @brief Returns the version of the DSM library
  /// @return The version of the DSM library
  auto const& version() { return DSF_VERSION; };
}  // namespace dsf

#include "headers/AdjacencyMatrix.hpp"
#include "headers/Agent.hpp"
#include "headers/RoadNetwork.hpp"
#include "headers/Itinerary.hpp"
#include "headers/Intersection.hpp"
#include "headers/TrafficLight.hpp"
#include "headers/Roundabout.hpp"
#include "headers/SparseMatrix.hpp"
#include "headers/Edge.hpp"
#include "headers/Street.hpp"
#include "headers/FirstOrderDynamics.hpp"
#include "utility/TypeTraits/is_node.hpp"
#include "utility/TypeTraits/is_street.hpp"
#include "utility/TypeTraits/is_numeric.hpp"

#endif
