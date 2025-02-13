
#include "../headers/AdjacencyMatrix.hpp"
#include "../utility/Logger.hpp"
#include <tbb/parallel_for_each.h>

#include <cassert>
#include <fstream>
#include <iostream>

namespace dsm {

  namespace test {
    std::vector<Id> offsets(const AdjacencyMatrix& adj) { return adj.m_rowOffsets; }

    std::vector<Id> indices(const AdjacencyMatrix& adj) { return adj.m_columnIndices; }
  }  // namespace test

  /*********************************************************************************
   * CONSTRUCTORS
   **********************************************************************************/
  AdjacencyMatrix::AdjacencyMatrix()
      : m_rowOffsets{std::vector<Id>(2, 0)}, m_columnIndices{}, m_n{1} {}
  AdjacencyMatrix::AdjacencyMatrix(std::string const& fileName) { read(fileName); }
  AdjacencyMatrix::AdjacencyMatrix(
      const std::unordered_map<Id, std::unique_ptr<Street>>& streets)
      : m_n{0} {
    const auto size = streets.size();
    std::vector<Id> rowSizes(size + 1, 0);  // Extra space for safety
    std::vector<Id> colIndexes(size);

    // First pass: Count rows and determine matrix dimensions
    for (const auto& pair : streets) {
      auto row = pair.second->source();
      auto col = pair.second->target();

      // Ensure valid row index
      if (row >= rowSizes.size()) {
        rowSizes.resize(row + 2, 0);  // Resize safely
      }
      assert(row + 1 < rowSizes.size());
      rowSizes[row + 1]++;

      // Track max row and column indices
      if (row >= m_n) {
        m_n = row + 1;
      }
      if (col >= m_n) {
        m_n = col + 1;
      }
    }

    // Compute row offsets using inclusive scan
    std::vector<Id> tempOffsets(m_n + 1, 0);
    std::inclusive_scan(
        rowSizes.begin(), rowSizes.begin() + m_n + 1, tempOffsets.begin());
    m_rowOffsets = tempOffsets;

    // Second pass: Assign column indices correctly
    std::vector<Id> currentOffset = tempOffsets;  // Copy tempOffsets for indexing
    for (const auto& pair : streets) {
      auto row = pair.second->source();
      auto col = pair.second->target();

      assert(row < currentOffset.size());
      assert(currentOffset[row] < colIndexes.size());

      colIndexes[currentOffset[row]] = col;
      ++currentOffset[row];  // Increment after assigning
    }

    m_columnIndices = std::move(colIndexes);
  }
  /*********************************************************************************
   * OPERATORS
   **********************************************************************************/
  bool AdjacencyMatrix::operator==(const AdjacencyMatrix& other) const {
    return (m_rowOffsets == other.m_rowOffsets) &&
           (m_columnIndices == other.m_columnIndices) && (m_n == other.m_n);
  }
  bool AdjacencyMatrix::operator()(Id row, Id col) const { return contains(row, col); }

  void AdjacencyMatrix::transpose() {
    std::vector<Id> newColumnIndices(m_columnIndices.size());
    std::vector<Id> newRowOffsets(m_n + 1, 0);

    // Count the number of elements in each column
    tbb::parallel_for_each(m_columnIndices.begin(), m_columnIndices.end(), [&](auto& x) {
      newRowOffsets[x + 1]++;
    });

    // Compute the row offsets using inclusive scan
    std::inclusive_scan(
        newRowOffsets.begin(), newRowOffsets.end(), newRowOffsets.begin());
    std::vector<Id> insertionOffsets = newRowOffsets;

    // 4. Populate the transposed matrix.
    // 'm_rowOffsets' and 'm_columnIndices' represent the original matrix (in CSR).
    for (size_t i = 0; i < m_n; ++i) {
      for (size_t j = m_rowOffsets[i]; j < m_rowOffsets[i + 1]; ++j) {
        // 'col' in the original matrix becomes the row in the transposed matrix.
        Id col = m_columnIndices[j];
        // Use insertionOffsets[col] as the next free slot for row 'col' in the transposed data.
        Id pos = insertionOffsets[col];
        newColumnIndices[pos] = i;
        insertionOffsets[col]++;
      }
    }

    m_columnIndices = std::move(newColumnIndices);
    m_rowOffsets = std::move(newRowOffsets);
  }
  /*********************************************************************************
   * METHODS
   **********************************************************************************/

  size_t AdjacencyMatrix::n() const { return m_n; }
  size_t AdjacencyMatrix::size() const { return m_columnIndices.size(); }

  void AdjacencyMatrix::insert(Id row, Id col) {
    if (row >= m_n) {
      m_n = row + 1;
    }
    if (col >= m_n) {
      m_n = col + 1;
    }

    // Ensure m_rowOffsets has at least m_nRows + 1 elements
    while (m_rowOffsets.size() <= m_n) {
      m_rowOffsets.push_back(m_rowOffsets.back());
    }

    assert(row + 1 < m_rowOffsets.size());
    // Increase row offsets for rows after the inserted row
    tbb::parallel_for_each(
        m_rowOffsets.begin() + row + 1, m_rowOffsets.end(), [](Id& x) { x++; });

    // Insert column index at the correct position
    auto const offset = m_rowOffsets[row + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + offset, col);
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
    assert(row + 1 < m_rowOffsets.size());
    const auto lowerOffset = m_rowOffsets[row];
    const auto upperOffset = m_rowOffsets[row + 1];
    std::vector<Id> rowVector(upperOffset - lowerOffset);

    std::copy(m_columnIndices.begin() + m_rowOffsets[row],
              m_columnIndices.begin() + m_rowOffsets[row + 1],
              rowVector.begin());
    return rowVector;
  }
  std::vector<Id> AdjacencyMatrix::getCol(Id col) const {
    std::vector<Id> colVector{};
    for (auto row = 0u; row < m_n; ++row) {
      assert(row + 1 < m_rowOffsets.size());
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
    m_rowOffsets = std::vector<Id>(2, 0);
    m_columnIndices.clear();
    m_n = 1;
  }
  void AdjacencyMatrix::clearRow(Id row) {
    assert(row + 1 < m_rowOffsets.size());
    const auto lowerOffset = m_rowOffsets[row];
    const auto upperOffset = m_rowOffsets[row + 1];
    m_columnIndices.erase(m_columnIndices.begin() + lowerOffset,
                          m_columnIndices.begin() + upperOffset);
    std::for_each(
        m_rowOffsets.begin() + row + 1,
        m_rowOffsets.end(),
        [upperOffset, lowerOffset](auto& x) { x -= upperOffset - lowerOffset; });
  }
  void AdjacencyMatrix::clearCol(Id col) {
    for (auto row = 0u; row < m_n; ++row) {
      assert(row + 1 < m_rowOffsets.size());
      const auto lowerOffset = m_rowOffsets[row];
      const auto upperOffset = m_rowOffsets[row + 1];
      auto it = std::find(m_columnIndices.begin() + lowerOffset,
                          m_columnIndices.begin() + upperOffset,
                          col);
      if (it != m_columnIndices.begin() + upperOffset) {
        m_columnIndices.erase(it);
        std::for_each(m_rowOffsets.begin() + row + 1,
                      m_rowOffsets.end(),
                      [upperOffset, lowerOffset](auto& x) { x--; });
      }
    }
  }

  std::vector<int> AdjacencyMatrix::getOutDegreeVector() const {
    auto degVector = std::vector<int>(m_n);
    std::adjacent_difference(
        m_rowOffsets.begin() + 1, m_rowOffsets.end(), degVector.begin());
    return degVector;
  }

  std::vector<int> AdjacencyMatrix::getInDegreeVector() const {
    auto degVector = std::vector<int>(m_n, 0);
    std::for_each(m_columnIndices.begin(),
                  m_columnIndices.end(),
                  [&degVector](const auto& x) { degVector[x]++; });
    return degVector;
  }

  void AdjacencyMatrix::read(std::string const& fileName) {
    std::ifstream inStream(fileName, std::ios::binary);
    if (!inStream.is_open()) {
      Logger::error(std::format("Could not open file {} for reading.", fileName));
    }
    inStream.read(reinterpret_cast<char*>(&m_n), sizeof(size_t));
    m_rowOffsets.resize(m_n + 1);
    inStream.read(reinterpret_cast<char*>(m_rowOffsets.data()),
                  m_rowOffsets.size() * sizeof(Id));
    m_columnIndices.resize(m_rowOffsets.back());
    inStream.read(reinterpret_cast<char*>(m_columnIndices.data()),
                  m_columnIndices.size() * sizeof(Id));
    inStream.close();
  }

  void AdjacencyMatrix::save(std::string const& fileName) const {
    std::ofstream outStream(fileName, std::ios::binary);
    if (!outStream.is_open()) {
      Logger::error(std::format("Could not open file {} for writing.", fileName));
    }
    outStream.write(reinterpret_cast<const char*>(&m_n), sizeof(size_t));
    outStream.write(reinterpret_cast<const char*>(m_rowOffsets.data()),
                    m_rowOffsets.size() * sizeof(Id));
    outStream.write(reinterpret_cast<const char*>(m_columnIndices.data()),
                    m_columnIndices.size() * sizeof(Id));
    outStream.close();
  }

}  // namespace dsm
