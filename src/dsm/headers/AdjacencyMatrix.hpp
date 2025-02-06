
#pragma once

#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "../headers/Street.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {

  class AdjacencyMatrix;

  namespace test {
    std::vector<Id> offsets(const AdjacencyMatrix& adj);
    std::vector<Id> indices(const AdjacencyMatrix& adj);
  }  // namespace test

  class AdjacencyMatrix {
  private:
    std::vector<Id> m_rowOffsets;
    std::vector<Id> m_columnIndices;
    size_t m_nRows;
    size_t m_nCols;

    friend std::vector<Id> test::offsets(const AdjacencyMatrix& adj);
    friend std::vector<Id> test::indices(const AdjacencyMatrix& adj);

  public:
    using value_type = Id;
    using difference_type = std::ptrdiff_t;
    using reference = Id&;
    using const_reference = const Id&;
    using iterator = std::vector<Id>::iterator;
    using const_iterator = std::vector<Id>::const_iterator;
    using reverse_iterator = std::vector<Id>::reverse_iterator;
    using const_reverse_iterator = std::vector<Id>::const_reverse_iterator;

    AdjacencyMatrix()
        : m_rowOffsets{std::vector<Id>(2, 0)},
          m_columnIndices{},
          m_nRows{1},
          m_nCols{0} {}
    AdjacencyMatrix(const std::unordered_map<Id, std::unique_ptr<Street>>& streets)
        : m_rowOffsets(streets.size()), m_nRows{0}, m_nCols{0} {
      auto edgeFirst = streets.begin();
      auto edgeLast = streets.end();
      const auto size = streets.size();
      std::vector<Id> rowSizes(size, 0);
      std::vector<Id> colIndexes(size);
      std::for_each(edgeFirst,
                    edgeLast,
                    [this, &rowSizes, &colIndexes, i = 0](const auto& pair) -> void {
                      auto row = pair.second->source();
                      rowSizes[row + 1]++;
                      if (row >= m_nRows)
                        m_nRows = row + 1;
                    });
      std::vector<Id> tempOffsets(m_nRows + 1, 0);
      tempOffsets[0] = 0;
      std::inclusive_scan(
          rowSizes.begin(), rowSizes.begin() + m_nRows + 1, tempOffsets.begin());
      m_rowOffsets = tempOffsets;

      std::for_each(edgeFirst,
                    edgeLast,
                    [this, &tempOffsets, &colIndexes, i = 0](const auto& pair) -> void {
                      auto row = pair.second->source();
                      auto col = pair.second->target();
                      if (col >= m_nCols)
                        m_nCols = col + 1;
                      colIndexes[tempOffsets[row]] = col;
                      ++tempOffsets[row];
                    });
      /* m_columnIndices = std::move(colIndexes); */
      m_columnIndices = std::move(colIndexes);
    }

    auto nRows() const { return m_nRows; }
    auto nCols() const { return m_nCols; }

    void insert(Id row, Id col);

    bool contains(Id row, Id col);
    bool operator()(Id row, Id col);

    std::vector<Id> getRow(Id row);
    std::vector<Id> getCol(Id col);

    std::vector<int> getInDegreeVector();
    std::vector<int> getOutDegreeVector();
  };
}  // namespace dsm
