
#include "../headers/AdjacencyMatrix.hpp"
#include "../utility/Logger.hpp"

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
      : m_rowOffsets{std::vector<Id>(2, 0)}, m_columnIndices{}, m_nRows{1}, m_nCols{0} {}
  AdjacencyMatrix::AdjacencyMatrix(std::string const& fileName) { read(fileName); }
  AdjacencyMatrix::AdjacencyMatrix(
      const std::unordered_map<Id, std::unique_ptr<Street>>& streets)
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
                    #ifndef NDEBUG
                    rowSizes.at(row + 1)++;
                    #else
                    rowSizes[row + 1]++;
                    #endif
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
                    #ifndef NDEBUG
                    colIndexes.at(tempOffsets.at(row)) = col;
                    ++tempOffsets.at(row);
                    #else
                    colIndexes[tempOffsets[row]] = col;
                    ++tempOffsets[row];
                    #endif
                  });
    m_columnIndices = std::move(colIndexes);
  }
  /*********************************************************************************
   * OPERATORS
   **********************************************************************************/
  bool AdjacencyMatrix::operator==(const AdjacencyMatrix& other) const {
    return (m_rowOffsets == other.m_rowOffsets) &&
           (m_columnIndices == other.m_columnIndices) && (m_nRows == other.m_nRows) &&
           (m_nCols == other.m_nCols);
  }
  bool AdjacencyMatrix::operator()(Id row, Id col) const { return contains(row, col); }

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
    #ifndef NDEBUG
    const auto offset = m_rowOffsets.at(row + 1) - 1;
    #else
    const auto offset = m_rowOffsets[row + 1] - 1;
    #endif
    m_columnIndices.insert(m_columnIndices.begin() + offset, col);
  }

  size_t AdjacencyMatrix::size() const { return m_columnIndices.size(); }

  bool AdjacencyMatrix::contains(Id row, Id col) const {
    if (row >= m_nRows or col >= m_nCols) {
      throw std::out_of_range("Row or column index out of range.");
    }

    auto itFirst = m_columnIndices.begin() + m_rowOffsets[row];
    auto itLast = m_columnIndices.begin() + m_rowOffsets[row + 1];
    return std::find(itFirst, itLast, col) != itLast;
  }

  std::vector<Id> AdjacencyMatrix::getRow(Id row) const {
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

  std::vector<std::pair<Id, Id>> AdjacencyMatrix::elements() const {
    std::vector<std::pair<Id, Id>> elements;
    for (auto row = 0u; row < m_nRows; ++row) {
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
    m_nRows = 1;
    m_nCols = 0;
  }
  void AdjacencyMatrix::clearRow(Id row) {
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
    for (auto row = 0u; row < m_nRows; ++row) {
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

  void AdjacencyMatrix::read(std::string const& fileName) {
    std::ifstream inStream(fileName, std::ios::binary);
    if (!inStream.is_open()) {
      Logger::error(std::format("Could not open file {} for reading.", fileName));
    }
    inStream.read(reinterpret_cast<char*>(&m_nRows), sizeof(size_t));
    inStream.read(reinterpret_cast<char*>(&m_nCols), sizeof(size_t));
    m_rowOffsets.resize(m_nRows + 1);
    m_columnIndices.resize(m_nCols);
    inStream.read(reinterpret_cast<char*>(m_rowOffsets.data()),
                  m_rowOffsets.size() * sizeof(Id));
    inStream.read(reinterpret_cast<char*>(m_columnIndices.data()),
                  m_columnIndices.size() * sizeof(Id));
    inStream.close();
  }

  void AdjacencyMatrix::save(std::string const& fileName) const {
    std::ofstream outStream(fileName, std::ios::binary);
    if (!outStream.is_open()) {
      Logger::error(std::format("Could not open file {} for writing.", fileName));
    }
    outStream.write(reinterpret_cast<const char*>(&m_nRows), sizeof(size_t));
    outStream.write(reinterpret_cast<const char*>(&m_nCols), sizeof(size_t));
    outStream.write(reinterpret_cast<const char*>(m_rowOffsets.data()),
                    m_rowOffsets.size() * sizeof(Id));
    outStream.write(reinterpret_cast<const char*>(m_columnIndices.data()),
                    m_columnIndices.size() * sizeof(Id));
    outStream.close();
  }

}  // namespace dsm
