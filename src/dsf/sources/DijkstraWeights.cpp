
#include "../headers/DijkstraWeights.hpp"
#include "../headers/RoadNetwork.hpp"

namespace dsf {

  namespace weight_functions {
    double streetLength(const RoadNetwork* graph,
                        Id edgeId,
                        [[maybe_unused]] double param) {
      return graph->edge(edgeId)->length();
    }

    double streetTime(const RoadNetwork* graph,
                      Id edgeId,
                      [[maybe_unused]] double param) {
      auto const& pStreet{graph->edge(edgeId)};
      const auto length{pStreet->length()};
      const auto speed{
          pStreet->maxSpeed() *
          (1. - param * pStreet->density(true))};  // param is the alpha parameter
      return length / speed;
    }

    double streetWeight(const RoadNetwork* graph,
                        Id edgeId,
                        [[maybe_unused]] double param) {
      auto const& pStreet{graph->edge(edgeId)};
      return pStreet->weight();
    }
  }  // namespace weight_functions

};  // namespace dsf
