
#include "../headers/Itinerary.hpp"

namespace dsf {
  Itinerary::Itinerary(Id id, Id destination) : m_id{id}, m_destination{destination} {}

  void Itinerary::setPath(AdjacencyMatrix path) {
    if (path.n() < m_destination) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("The path's dimension must be "
                      "greater than the itinerary's destination ({}).",
                      path.n(),
                      m_destination)));
    }
    m_path = std::make_unique<AdjacencyMatrix>(std::move(path));
  }

  Id Itinerary::id() const { return m_id; }
  Id Itinerary::destination() const { return m_destination; }
  std::unique_ptr<AdjacencyMatrix> const& Itinerary::path() const { return m_path; }

};  // namespace dsf
