
#include "Itinerary.hpp"

namespace dsf::mobility {
  Itinerary::Itinerary(Id const& source, Id const& destination) : m_source{source}, m_destination{destination} {}

  void Itinerary::setPaths(std::multimap<double, std::vector<Id>> const& paths) {
    m_paths.clear();
    for (auto const& [cost, path] : paths) {
      m_paths.emplace(cost, std::make_shared<std::vector<Id>>(path));
    }
  }

};  // namespace dsf::mobility
