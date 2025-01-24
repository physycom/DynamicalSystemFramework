
#include "Itinerary.hpp"

namespace dsm {
  Itinerary::Itinerary(Id id, Id destination) : m_id{id}, m_destination{destination} {}

  void Itinerary::setPath(SparseMatrix<bool> path) {
    if (path.getRowDim() != path.getColDim()) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("The path's row ({}) and column ({}) dimensions must be equal.",
                      path.getRowDim(),
                      path.getColDim())));
    }
    if (path.getRowDim() < m_destination) {
      throw std::invalid_argument(Logger::buildExceptionMessage(
          std::format("The path's row ({}) and column ({}) dimensions must be "
                      "greater than the itinerary's destination ({}).",
                      path.getRowDim(),
                      path.getColDim(),
                      m_destination)));
    }
    m_path = std::move(path);
  }

};  // namespace dsm
