
#include "AdjacencyMatrix.hpp"

#include <cassert>
#include <fstream>
#include <iostream>

#include <tbb/parallel_for_each.h>
#include <spdlog/spdlog.h>

namespace dsf {

  namespace test {
    std::vector<Id> offsets(const AdjacencyMatrix& adj) { return adj.m_rowOffsets; }

    std::vector<Id> indices(const AdjacencyMatrix& adj) { return adj.m_columnIndices; }
  }  // namespace test

  /*********************************************************************************
   * CONSTRUCTORS
   **********************************************************************************/
  AdjacencyMatrix::AdjacencyMatrix()
      : m_rowOffsets{std::vector<Id>(1, 0)},
        m_columnIndices{},
        m_colOffsets{std::vector<Id>(1, 0)},
        m_rowIndices{},
        m_n{0} {}
  AdjacencyMatrix::AdjacencyMatrix(std::string const& fileName) { read(fileName); }
  /*********************************************************************************
   * OPERATORS
   **********************************************************************************/
  bool AdjacencyMatrix::operator==(const AdjacencyMatrix& other) const {
    return (m_rowOffsets == other.m_rowOffsets) &&
           (m_columnIndices == other.m_columnIndices) && (m_n == other.m_n);
  }
  bool AdjacencyMatrix::operator()(Id row, Id col) const { return contains(row, col); }
  /*********************************************************************************
   * METHODS
   **********************************************************************************/

  size_t AdjacencyMatrix::n() const { return m_n; }
  size_t AdjacencyMatrix::size() const {
    assert(m_columnIndices.size() == m_rowIndices.size());
    return m_columnIndices.size();
  }
  bool AdjacencyMatrix::empty() const {
    assert(m_columnIndices.size() == m_rowIndices.size());
    return m_columnIndices.empty();
  }

  void AdjacencyMatrix::insert(Id row, Id col) {
    m_n = std::max(m_n, static_cast<size_t>(row + 1));
    m_n = std::max(m_n, static_cast<size_t>(col + 1));

    // Ensure rowOffsets and colOffsets have at least m_n + 1 elements
    while (m_rowOffsets.size() <= m_n) {
      m_rowOffsets.push_back(m_rowOffsets.back());
    }
    while (m_colOffsets.size() <= m_n) {
      m_colOffsets.push_back(m_colOffsets.back());
    }

    assert(row + 1 < m_rowOffsets.size());
    assert(col + 1 < m_colOffsets.size());

    // Increase row offsets for rows after the inserted row (CSR)
    tbb::parallel_for_each(
        m_rowOffsets.begin() + row + 1, m_rowOffsets.end(), [](Id& x) { x++; });

    // Increase column offsets for columns after the inserted column (CSC)
    tbb::parallel_for_each(
        m_colOffsets.begin() + col + 1, m_colOffsets.end(), [](Id& x) { x++; });

    // Insert column index at the correct position for CSR
    auto csrOffset = m_rowOffsets[row + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + csrOffset, col);

    // Insert row index at the correct position for CSC
    auto cscOffset = m_colOffsets[col + 1] - 1;
    m_rowIndices.insert(m_rowIndices.begin() + cscOffset, row);
  }

  bool AdjacencyMatrix::contains(Id row, Id col) const {
    if (row >= m_n or col >= m_n) {
      throw std::out_of_range("Row or column index out of range.");
    }
    assert(row + 1 < m_rowOffsets.size());
    auto itFirst = m_columnIndices.begin() + m_rowOffsets[row];
    auto itLast = m_columnIndices.begin() + m_rowOffsets[row + 1];
    return std::find(itFirst, itLast, col) != itLast;
  }

  std::vector<Id> AdjacencyMatrix::getRow(Id row) const {
    if (row + 1 >= m_rowOffsets.size()) {
      throw std::out_of_range(
          std::format("Row index {} out of range [0, {}[.", row, m_n - 1));
    }
    const auto lowerOffset = m_rowOffsets[row];
    const auto upperOffset = m_rowOffsets[row + 1];
    std::vector<Id> rowVector(upperOffset - lowerOffset);

    std::copy(m_columnIndices.begin() + m_rowOffsets[row],
              m_columnIndices.begin() + m_rowOffsets[row + 1],
              rowVector.begin());
    return rowVector;
  }
  std::vector<Id> AdjacencyMatrix::getCol(Id col) const {
    assert(col + 1 < m_colOffsets.size());
    const auto lowerOffset = m_colOffsets[col];
    const auto upperOffset = m_colOffsets[col + 1];
    std::vector<Id> colVector(upperOffset - lowerOffset);

    std::copy(m_rowIndices.begin() + lowerOffset,
              m_rowIndices.begin() + upperOffset,
              colVector.begin());
    return colVector;
  }

  std::vector<std::pair<Id, Id>> AdjacencyMatrix::elements() const {
    std::vector<std::pair<Id, Id>> elements;
    for (auto row = 0u; row < m_n; ++row) {
      assert(row + 1 < m_rowOffsets.size());
      const auto lowerOffset = m_rowOffsets[row];
      const auto upperOffset = m_rowOffsets[row + 1];
      for (auto i = lowerOffset; i < upperOffset; ++i) {
        elements.emplace_back(row, m_columnIndices[i]);
      }
    }
    return elements;
  }

  void AdjacencyMatrix::clear() {
    m_rowOffsets = std::vector<Id>(1, 0);
    m_colOffsets = std::vector<Id>(1, 0);
    m_columnIndices.clear();
    m_rowIndices.clear();
    m_n = 0;
  }
  void AdjacencyMatrix::clearRow(Id row) {
    // CSR: Clear row in column indices
    assert(row + 1 < m_rowOffsets.size());
    const auto lowerOffset = m_rowOffsets[row];
    const auto upperOffset = m_rowOffsets[row + 1];
    m_columnIndices.erase(m_columnIndices.begin() + lowerOffset,
                          m_columnIndices.begin() + upperOffset);
    std::transform(
        DSF_EXECUTION m_rowOffsets.begin() + row + 1,
        m_rowOffsets.end(),
        m_rowOffsets.begin() + row + 1,
        [upperOffset, lowerOffset](auto& x) { return x - (upperOffset - lowerOffset); });

    // CSC: Clear the corresponding rows from column offsets
    for (auto col = 0u; col < m_n; ++col) {
      assert(col + 1 < m_colOffsets.size());
      const auto colLowerOffset = m_colOffsets[col];
      const auto colUpperOffset = m_colOffsets[col + 1];
      auto it = std::find(m_rowIndices.begin() + colLowerOffset,
                          m_rowIndices.begin() + colUpperOffset,
                          row);
      if (it != m_rowIndices.begin() + colUpperOffset) {
        // Remove row from rowIndices and update the rowOffsets
        m_rowIndices.erase(it);
        // Decrement the offsets for rows after the current row
        std::transform(DSF_EXECUTION m_colOffsets.begin() + col + 1,
                       m_colOffsets.end(),
                       m_colOffsets.begin() + col + 1,
                       [](auto& x) { return x - 1; });
      }
    }
  }

  void AdjacencyMatrix::clearCol(Id col) {
    // CSR: Clear column in row indices
    for (auto row = 0u; row < m_n; ++row) {
      assert(row + 1 < m_rowOffsets.size());
      const auto lowerOffset = m_rowOffsets[row];
      const auto upperOffset = m_rowOffsets[row + 1];
      auto it = std::find(m_columnIndices.begin() + lowerOffset,
                          m_columnIndices.begin() + upperOffset,
                          col);
      if (it != m_columnIndices.begin() + upperOffset) {
        m_columnIndices.erase(it);
        // Decrement the offsets for rows after the current row
        std::transform(DSF_EXECUTION m_rowOffsets.begin() + row + 1,
                       m_rowOffsets.end(),
                       m_rowOffsets.begin() + row + 1,
                       [](auto& x) { return x - 1; });
      }
    }

    // CSC: Clear the column from row indices and update column offsets
    assert(col + 1 < m_colOffsets.size());
    const auto lowerOffset = m_colOffsets[col];
    const auto upperOffset = m_colOffsets[col + 1];
    m_rowIndices.erase(m_rowIndices.begin() + lowerOffset,
                       m_rowIndices.begin() + upperOffset);

    // Adjust column offsets accordingly
    std::transform(
        DSF_EXECUTION m_colOffsets.begin() + col + 1,
        m_colOffsets.end(),
        m_colOffsets.begin() + col + 1,
        [upperOffset, lowerOffset](auto& x) { return x - (upperOffset - lowerOffset); });
  }

  std::vector<int> AdjacencyMatrix::getOutDegreeVector() const {
    auto degVector = std::vector<int>(m_n);
    std::adjacent_difference(
        m_rowOffsets.begin() + 1, m_rowOffsets.end(), degVector.begin());
    return degVector;
  }
  std::vector<int> AdjacencyMatrix::getInDegreeVector() const {
    auto degVector = std::vector<int>(m_n);
    std::adjacent_difference(
        m_colOffsets.begin() + 1, m_colOffsets.end(), degVector.begin());
    return degVector;
  }

  void AdjacencyMatrix::read(std::string const& fileName) {
    std::ifstream inStream(fileName, std::ios::binary);
    if (!inStream.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    inStream.read(reinterpret_cast<char*>(&m_n), sizeof(size_t));
    m_rowOffsets.resize(m_n + 1);
    inStream.read(reinterpret_cast<char*>(m_rowOffsets.data()),
                  m_rowOffsets.size() * sizeof(Id));
    m_columnIndices.resize(m_rowOffsets.back());
    inStream.read(reinterpret_cast<char*>(m_columnIndices.data()),
                  m_columnIndices.size() * sizeof(Id));
    inStream.close();
    // Initialize CSC format variables
    m_colOffsets.resize(m_n + 1, 0);
    m_rowIndices.resize(m_columnIndices.size());

    // Compute CSC from CSR
    std::vector<Id> colSizes(m_n, 0);

    // Count occurrences of each column index
    for (const auto& col : m_columnIndices) {
      colSizes[col]++;
    }

    // Compute column offsets using an inclusive scan
    std::inclusive_scan(colSizes.begin(), colSizes.end(), m_colOffsets.begin() + 1);

    // Fill CSC row indices
    std::vector<Id> currentOffset = m_colOffsets;
    for (Id row = 0; row < m_n; ++row) {
      for (Id i = m_rowOffsets[row]; i < m_rowOffsets[row + 1]; ++i) {
        Id col = m_columnIndices[i];
        m_rowIndices[currentOffset[col]++] = row;
      }
    }
  }

  void AdjacencyMatrix::save(std::string const& fileName) const {
    std::ofstream outStream(fileName, std::ios::binary);
    if (!outStream.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }
    outStream.write(reinterpret_cast<const char*>(&m_n), sizeof(size_t));
    outStream.write(reinterpret_cast<const char*>(m_rowOffsets.data()),
                    m_rowOffsets.size() * sizeof(Id));
    outStream.write(reinterpret_cast<const char*>(m_columnIndices.data()),
                    m_columnIndices.size() * sizeof(Id));
    outStream.close();
  }

}  // namespace dsf
