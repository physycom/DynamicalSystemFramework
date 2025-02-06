
#include "../headers/AdjacencyMatrix.hpp"
#include <iostream>

namespace dsm {

  namespace test {
    std::vector<Id> offsets(const AdjacencyMatrix& adj) { return adj.m_rowOffsets; }

    std::vector<Id> indices(const AdjacencyMatrix& adj) { return adj.m_columnIndices; }
  }  // namespace test

  void AdjacencyMatrix::insert(Id row, Id col) {
    if (row > m_rowOffsets.size() - 2) {
      for (auto i = 0ul; i < row - m_rowOffsets.size() + 2; ++i) {
        m_rowOffsets.push_back(m_rowOffsets.back() + 1);
      }
      m_nRows = row + 1;
    } else {
      std::for_each(
          m_rowOffsets.begin() + row + 1, m_rowOffsets.end(), [](Id& x) { x++; });
    }

    if (col >= m_nCols)
      m_nCols = col + 1;
    const auto offset = m_rowOffsets[row + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + offset, col);
  }

  bool AdjacencyMatrix::contains(Id row, Id col) {
    if (row >= m_nRows or col >= m_nCols) {
      throw std::out_of_range("Row or column index out of range.");
    }

    iterator itFirst = m_columnIndices.begin() + m_rowOffsets[row];
    iterator itLast = m_columnIndices.begin() + m_rowOffsets[row + 1];
    return std::find(itFirst, itLast, col) != itLast;
  }
  bool AdjacencyMatrix::operator()(Id row, Id col) { return contains(row, col); }

  std::vector<Id> AdjacencyMatrix::getRow(Id row) {
    const auto lowerOffset = m_rowOffsets[row];
    const auto upperOffset = m_rowOffsets[row + 1];
    std::vector<Id> rowVector(upperOffset - lowerOffset);

    std::copy(m_columnIndices.begin() + m_rowOffsets[row],
              m_columnIndices.begin() + m_rowOffsets[row + 1],
              rowVector.begin());
    return rowVector;
  }
  std::vector<Id> AdjacencyMatrix::getCol(Id col) {
    std::vector<Id> colVector{};
    for (auto row = 0u; row < m_nRows; ++row) {
      const auto lowerOffset = m_rowOffsets[row];
      const auto upperOffset = m_rowOffsets[row + 1];
      if (std::find(m_columnIndices.begin() + lowerOffset,
                    m_columnIndices.begin() + upperOffset,
                    col) != m_columnIndices.begin() + upperOffset) {
        colVector.push_back(row);
      }
    }
    return colVector;
  }

  std::vector<int> AdjacencyMatrix::getOutDegreeVector() {
    auto degVector = std::vector<int>(m_nRows);
    std::adjacent_difference(
        m_rowOffsets.begin() + 1, m_rowOffsets.end(), degVector.begin());
    return degVector;
  }

  std::vector<int> AdjacencyMatrix::getInDegreeVector() {
    auto degVector = std::vector<int>(m_nCols, 0);
    std::for_each(m_columnIndices.begin(),
                  m_columnIndices.end(),
                  [&degVector](const auto& x) { degVector[x]++; });
    return degVector;
  }

}  // namespace dsm
