#ifndef dsm_hpp
#define dsm_hpp

#include <cstdint>
#include <format>

static constexpr uint8_t DSM_VERSION_MAJOR = 2;
static constexpr uint8_t DSM_VERSION_MINOR = 5;
static constexpr uint8_t DSM_VERSION_PATCH = 12;

static auto const DSM_VERSION =
    std::format("{}.{}.{}", DSM_VERSION_MAJOR, DSM_VERSION_MINOR, DSM_VERSION_PATCH);

namespace dsm {
  /// @brief Returns the version of the DSM library
  /// @return The version of the DSM library
  auto const& version() { return DSM_VERSION; };
}  // namespace dsm

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
