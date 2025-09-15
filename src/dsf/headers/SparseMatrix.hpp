#pragma once

#include <algorithm>
#include <cassert>
#include <format>
#include <fstream>
#include <map>  // A flat map would be better
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <iostream>

#include "../utility/Typedef.hpp"

namespace dsf {
  /// @brief The SparseMatrix class represents a sparse square matrix in CSR format.
  /// @details This implementations is optimized for row access.
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  class SparseMatrix {
    typedef bool axis_t;

  private:
    // CSR format
    std::vector<Id> m_rowOffsets;
    std::vector<Id> m_columnIndices;
    std::vector<T> m_values;
    size_t m_n;
    T m_TdefaultValue = static_cast<T>(0);

  public:
    /// @brief Construct a new SparseMatrix object
    SparseMatrix();
    /// @brief Construct a new SparseMatrix object using the @ref read method
    /// @param fileName The name of the file containing the sparse matrix
    /// @param fileType The type of the file (default is "bin")
    SparseMatrix(std::string const& fileName, std::string const& fileType = "bin");
    /// @brief Copy constructor
    /// @param other The SparseMatrix to copy
    SparseMatrix(const SparseMatrix& other) = default;
    /// @brief Move constructor
    /// @param other The SparseMatrix to move
    SparseMatrix(SparseMatrix&& other) = default;

    /// @brief Returns the element at the specified row and column
    /// @param row The row index of the element
    /// @param col The column index of the element
    /// @return The element at the specified row and column
    T operator()(Id row, Id col) const;
    SparseMatrix& operator=(const SparseMatrix& other) = default;
    SparseMatrix& operator=(SparseMatrix&& other) = default;
    bool operator==(const SparseMatrix& other) const;

    /// @brief Returns the number of (non-zero) elements in the matrix
    /// @return The number of elements in the matrix
    inline size_t size() const noexcept { return m_values.size(); }
    /// @brief Returns true if the matrix is empty
    /// @return true if the matrix is empty, false otherwise
    inline bool empty() const noexcept { return m_values.empty(); }
    /// @brief Returns the number of rows (columns) in the matrix
    /// @return The number of rows (columns) in the matrix
    inline size_t n() const noexcept { return m_n; }
    /// @brief Returns a row of the matrix as a map of column indices and values
    /// @param idRow The row index
    /// @return A map of column indices and values for the specified row
    std::map<Id, T> row(Id idRow) const;
    /// @brief Returns a column of the matrix as a map of row indices and values
    /// @param idCol The column index
    /// @return A map of row indices and values for the specified column
    std::map<Id, T> col(Id idCol) const;

    /// @brief Set the default value of the matrix (default is 0)
    /// @param value The default value of the matrix
    inline void setDefaultValue(T value) { m_TdefaultValue = value; }
    /// @brief Insert a new element of given value at the specified row and column
    /// @param row The row index of the element
    /// @param col The column index of the element
    /// @param value The value of the element
    /// @details This function actually inserts element \f$a_{ij}\f$ of the sparse matrix.
    ///   Where \f$i\f$ is the row index and \f$j\f$ is the column index.
    ///   The function will automatically resize the matrix to fit the new element.
    void insert(Id row, Id col, T value);

    /// @brief Normalize the rows of the matrix
    /// @param axis If 1, normalize the rows, otherwise (0) normalize the columns
    /// @param value The value to normalize the rows to (default is 1)
    /// @details The function will normalize the rows of the matrix to have the sum equal to the given value.
    void normalize(const axis_t axis = true, const T value = static_cast<T>(1));

    /// @brief Read the sparse matrix from a binary file
    /// @param fileName The name of the file containing the sparse matrix
    /// @param fileType The type of the file (default is "bin")
    /// @param mapping A mapping of the indices from strings to 0 -> N-1
    /// @throw std::runtime_error if the file cannot be opened
    void read(std::string const& fileName,
              std::string const& fileType = "bin",
              std::unordered_map<std::string, Id> const& mapping = {});
    /// @brief Save the sparse matrix to a binary file
    /// @param fileName The name of the file to save the sparse matrix
    /// @throw std::runtime_error if the file cannot be opened
    void save(std::string const& fileName) const;
  };

  /*********************************************************************************
   * CONSTRUCTORS
   **********************************************************************************/
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  SparseMatrix<T>::SparseMatrix()
      : m_rowOffsets{std::vector<Id>(2, 0)}, m_columnIndices{}, m_values{}, m_n{1} {}
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  SparseMatrix<T>::SparseMatrix(std::string const& fileName, std::string const& fileType)
      : m_rowOffsets{std::vector<Id>(2, 0)}, m_columnIndices{}, m_values{}, m_n{1} {
    read(fileName, fileType);
  }
  /*********************************************************************************
     * OPERATORS
     **********************************************************************************/
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  T SparseMatrix<T>::operator()(Id row, Id col) const {
    if (row >= m_n || col >= m_n) {
      throw std::out_of_range("Row or column index out of range.");
    }
    assert(row + 1 < m_rowOffsets.size());
    auto itFirst = m_columnIndices.begin() + m_rowOffsets[row];
    auto itLast = m_columnIndices.begin() + m_rowOffsets[row + 1];
    auto it = std::find(itFirst, itLast, col);
    if (it == itLast) {
      return m_TdefaultValue;  // Return default value if not found
    }
    size_t const index = m_rowOffsets[row] + std::distance(itFirst, it);
    assert(index < m_values.size());
    return m_values[index];
  }
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  bool SparseMatrix<T>::operator==(const SparseMatrix& other) const {
    if (m_n != other.m_n) {
      return false;
    }
    if (m_rowOffsets != other.m_rowOffsets) {
      return false;
    }
    if (m_columnIndices != other.m_columnIndices) {
      return false;
    }
    if (m_values != other.m_values) {
      return false;
    }
    return true;
  }
  /*********************************************************************************
     * METHODS
     **********************************************************************************/
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  void SparseMatrix<T>::insert(Id row, Id col, T value) {
    m_n = std::max(m_n, static_cast<size_t>(row + 1));
    m_n = std::max(m_n, static_cast<size_t>(col + 1));

    // Ensure rowOffsets have at least m_n + 1 elements
    while (m_rowOffsets.size() <= m_n) {
      m_rowOffsets.push_back(m_rowOffsets.back());
    }

    assert(row + 1 < m_rowOffsets.size());

    // Increase row offsets for rows after the inserted row
    std::for_each(DSF_EXECUTION m_rowOffsets.begin() + row + 1,
                  m_rowOffsets.end(),
                  [](Id& x) { x++; });

    // Insert column index at the correct position
    auto csrOffset = m_rowOffsets[row + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + csrOffset, col);
    m_values.insert(m_values.begin() + csrOffset, value);
  }

  template <typename T>
    requires(std::is_arithmetic_v<T>)
  void SparseMatrix<T>::normalize(const axis_t axis, const T value) {
    if (axis) {
      // Normalize rows
      for (Id row = 0; row + 1 < m_rowOffsets.size(); ++row) {
        auto lowerOffset = m_rowOffsets[row];
        auto upperOffset = m_rowOffsets[row + 1];
        auto const sum = std::reduce(DSF_EXECUTION m_values.begin() + lowerOffset,
                                     m_values.begin() + upperOffset,
                                     static_cast<T>(0));
        if (sum != static_cast<T>(0)) {
          auto const factor = value / sum;
          for (auto i = lowerOffset; i < upperOffset; ++i) {
            m_values[i] *= factor;
          }
        }
      }
    } else {
      // Normalize columns
      for (Id col = 0; col < m_n; ++col) {
        T sum = static_cast<T>(0);
        for (Id row = 0; row + 1 < m_rowOffsets.size(); ++row) {
          auto lowerOffset = m_rowOffsets[row];
          auto upperOffset = m_rowOffsets[row + 1];
          for (auto i = lowerOffset; i < upperOffset; ++i) {
            if (m_columnIndices[i] == col) {
              sum += m_values[i];
            }
          }
        }
        if (sum != static_cast<T>(0)) {
          T factor = value / sum;
          for (Id row = 0; row + 1 < m_rowOffsets.size(); ++row) {
            auto lowerOffset = m_rowOffsets[row];
            auto upperOffset = m_rowOffsets[row + 1];
            for (auto i = lowerOffset; i < upperOffset; ++i) {
              if (m_columnIndices[i] == col) {
                m_values[i] *= factor;
              }
            }
          }
        }
      }
    }
  }

  template <typename T>
    requires(std::is_arithmetic_v<T>)
  std::map<Id, T> SparseMatrix<T>::row(Id idRow) const {
    if (idRow >= m_n) {
      throw std::out_of_range(std::format("Row index ({}) out of range.", idRow));
    }
    std::map<Id, T> rowMap;
    if (idRow + 1 >= m_rowOffsets.size()) {
      return rowMap;
    }
    const auto lowerOffset = m_rowOffsets[idRow];
    const auto upperOffset = m_rowOffsets[idRow + 1];
    for (auto i = lowerOffset; i < upperOffset; ++i) {
      rowMap[m_columnIndices[i]] = m_values[i];
    }
    return rowMap;
  }
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  std::map<Id, T> SparseMatrix<T>::col(Id idCol) const {
    if (idCol >= m_n) {
      throw std::out_of_range(std::format("Column index ({}) out of range.", idCol));
    }
    std::map<Id, T> colMap;
    for (Id row = 0; row + 1 < m_rowOffsets.size(); ++row) {
      auto lowerOffset = m_rowOffsets[row];
      auto upperOffset = m_rowOffsets[row + 1];

      for (auto i = lowerOffset; i < upperOffset; ++i) {
        if (m_columnIndices[i] == idCol) {
          colMap[row] = m_values[i];
          break;  // Since column indices are assumed to be sorted within a row
        }
      }
    }
    return colMap;
  }
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  void SparseMatrix<T>::read(std::string const& fileName,
                             std::string const& fileType,
                             std::unordered_map<std::string, Id> const& mapping) {
    // Binary file
    if (fileType == "bin") {
      std::ifstream inStream(fileName, std::ios::binary);
      if (!inStream.is_open()) {
        throw std::runtime_error(
            std::format("Could not open file \'{}\' for reading.", fileName));
      }
      inStream.read(reinterpret_cast<char*>(&m_n), sizeof(size_t));
      m_rowOffsets.resize(m_n + 1);
      inStream.read(reinterpret_cast<char*>(m_rowOffsets.data()),
                    m_rowOffsets.size() * sizeof(Id));
      m_columnIndices.resize(m_rowOffsets.back());
      inStream.read(reinterpret_cast<char*>(m_columnIndices.data()),
                    m_columnIndices.size() * sizeof(Id));
      m_values.resize(m_rowOffsets.back());
      inStream.read(reinterpret_cast<char*>(m_values.data()),
                    m_values.size() * sizeof(T));
      inStream.close();
      return;
    }
    // CSV (; separated) file
    if (fileType == "csv") {
      std::ifstream inStream(fileName);
      if (!inStream.is_open()) {
        throw std::runtime_error(
            std::format("Could not open file \'{}\' for reading.", fileName));
      }
      std::string line;
      while (std::getline(inStream, line)) {
        std::istringstream iss(line);
        std::string strRow, strCol, strValue;
        if (!std::getline(iss, strRow, ';') || !std::getline(iss, strCol, ';') ||
            !std::getline(iss, strValue)) {
          throw std::runtime_error(
              std::format("Malformed line in file '{}': {}", fileName, line));
        }
        try {
          auto const rowId =
              mapping.empty() ? static_cast<Id>(std::stoul(strRow)) : mapping.at(strRow);
          auto const colId =
              mapping.empty() ? static_cast<Id>(std::stoul(strCol)) : mapping.at(strCol);

          std::istringstream valueStream(strValue);
          T value;
          if (!(valueStream >> value)) {
            throw std::runtime_error("Invalid value format");
          }

          insert(rowId, colId, value);
        } catch (const std::exception& e) {
          throw std::runtime_error(
              std::format("Error parsing line in file '{}': {}\nLine: '{}'",
                          fileName,
                          e.what(),
                          line));
        }
      }
      inStream.close();
      return;
    }
    throw std::runtime_error(std::format("File type \'{}\' not supported.", fileType));
  }
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  void SparseMatrix<T>::save(std::string const& fileName) const {
    std::ofstream outStream(fileName, std::ios::binary);
    if (!outStream.is_open()) {
      throw std::runtime_error(
          std::format("Could not open file \'{}\' for writing.", fileName));
    }
    outStream.write(reinterpret_cast<const char*>(&m_n), sizeof(size_t));
    outStream.write(reinterpret_cast<const char*>(m_rowOffsets.data()),
                    m_rowOffsets.size() * sizeof(Id));
    outStream.write(reinterpret_cast<const char*>(m_columnIndices.data()),
                    m_columnIndices.size() * sizeof(Id));
    outStream.write(reinterpret_cast<const char*>(m_values.data()),
                    m_values.size() * sizeof(T));
    outStream.close();
  }
}  // namespace dsf