#pragma once

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

#include <tbb/parallel_for_each.h>

#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The SparseMatrix class represents a sparse square matrix in CSR format.
  /// @details This implementations is optimized for row access.
  template <typename T>
    requires(std::is_arithmetic_v<T>)
  class SparseMatrix {
  private:
    // CSR format
    std::vector<Id> m_rowOffsets;
    std::vector<Id> m_columnIndices;
    std::vector<T> m_values;
    size_t m_n;

  public:
    /// @brief Construct a new SparseMatrix object
    SparseMatrix();
    /// @brief Construct a new SparseMatrix object using the @ref read method
    /// @param fileName The name of the file containing the sparse matrix
    SparseMatrix(std::string const& fileName);

    T operator()(Id row, Id col) const;

    /// @brief Returns the number of (non-zero) elements in the matrix
    /// @return The number of elements in the matrix
    size_t size() const { return m_values.size(); }
    /// @brief Returns true if the matrix is empty
    /// @return true if the matrix is empty, false otherwise
    bool empty() const { return m_values.empty(); }
    /// @brief Returns the number of rows (columns) in the matrix
    /// @return The number of rows (columns) in the matrix
    size_t n() const { return m_n; }

    void insert(Id row, Id col, T value);

    void read(std::string const& fileName);

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
  SparseMatrix<T>::SparseMatrix(std::string const& fileName) {
    read(fileName);
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
      return static_cast<T>(0);  // Return default value if not found
    }
    size_t const index = std::distance(itFirst, it);
    assert(index < m_values.size());
    return m_values[index];
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
    tbb::parallel_for_each(
        m_rowOffsets.begin() + row + 1, m_rowOffsets.end(), [](Id& x) { x++; });

    // Insert column index at the correct position
    auto csrOffset = m_rowOffsets[row + 1] - 1;
    m_columnIndices.insert(m_columnIndices.begin() + csrOffset, col);
    m_values.insert(m_values.begin() + csrOffset, value);
  }
}  // namespace dsm