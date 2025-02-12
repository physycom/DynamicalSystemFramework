
#pragma once

#include <algorithm>
#include <numeric>
#include <string>
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

    AdjacencyMatrix();
    AdjacencyMatrix(std::string const& fileName);
    AdjacencyMatrix(const std::unordered_map<Id, std::unique_ptr<Street>>& streets);

    bool operator==(const AdjacencyMatrix& other) const;
    bool operator()(Id row, Id col);

    auto nRows() const { return m_nRows; }
    auto nCols() const { return m_nCols; }

    void insert(Id row, Id col);

    bool contains(Id row, Id col);

    std::vector<Id> getRow(Id row);
    std::vector<Id> getCol(Id col);

    std::vector<int> getInDegreeVector();
    std::vector<int> getOutDegreeVector();

    void read(std::string const& fileName);
    void save(std::string const& fileName) const;
  };
}  // namespace dsm
