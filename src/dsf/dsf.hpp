#ifndef dsf_hpp
#define dsf_hpp

#include <cstdint>
#include <format>

static constexpr uint8_t DSF_VERSION_MAJOR = 4;
static constexpr uint8_t DSF_VERSION_MINOR = 4;
static constexpr uint8_t DSF_VERSION_PATCH = 2;

static auto const DSF_VERSION =
    std::format("{}.{}.{}", DSF_VERSION_MAJOR, DSF_VERSION_MINOR, DSF_VERSION_PATCH);

namespace dsf {
  /// @brief Returns the version of the DSF library
  /// @return The version of the DSF library
  auto const& version() { return DSF_VERSION; };
}  // namespace dsf

#include "base/AdjacencyMatrix.hpp"
#include "base/Edge.hpp"
#include "base/SparseMatrix.hpp"
#include "mobility/Agent.hpp"
#include "mobility/FirstOrderDynamics.hpp"
#include "mobility/Intersection.hpp"
#include "mobility/Itinerary.hpp"
#include "mobility/RoadNetwork.hpp"
#include "mobility/Roundabout.hpp"
#include "mobility/Street.hpp"
#include "mobility/TrafficLight.hpp"
#include "mdt/TrajectoryCollection.hpp"
#include "utility/TypeTraits/is_node.hpp"
#include "utility/TypeTraits/is_street.hpp"
#include "utility/TypeTraits/is_numeric.hpp"

#endif
