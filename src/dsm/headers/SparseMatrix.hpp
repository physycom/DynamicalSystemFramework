/// @file       /src/dsm/headers/SparseMatrix.hpp
/// @brief      Defines the SparseMatrix class.
///
/// @details    This file contains the definition of the SparseMatrix class.
///             The SparseMatrix class represents a sparse matrix. It is templated by the type
///             of the matrix's index and the type of the matrix's value.
///             The matrix's index must be an unsigned integral type.

#pragma once

#include <concepts>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <format>

#include "../utility/Logger.hpp"
#include "../utility/Typedef.hpp"

namespace dsm {
  /// @brief The SparseMatrix class represents a sparse matrix.
  /// @tparam Id The type of the matrix's index. It must be an unsigned integral type.
  /// @tparam T The type of the matrix's value.
  template <typename T>
  class SparseMatrix {
    std::unordered_map<Id, T> _matrix;
    Id _rows, _cols;
    T _defaultReturn;

  public:
    SparseMatrix();

    /// @brief SparseMatrix constructor
    /// @param rows number of rows
    /// @param cols number of columns
    /// @throw std::invalid_argument if rows or cols are < 0
    SparseMatrix(Id rows, Id cols);

    /// @brief SparseMatrix constructor - colum
    /// @param index number of rows
    /// @throw std::invalid_argument if index is < 0
    explicit SparseMatrix(Id index);

    /// @brief insert a value in the matrix
    /// @param i row index
    /// @param j column index
    /// @param value value to insert
    /// @throw std::out_of_range if the index is out of range
    void insert(Id i, Id j, T value);

    /// @brief insert a value in the matrix
    /// @param i index
    /// @param value value to insert
    /// @throw std::out_of_range if the index is out of range
    void insert(Id i, T value);

    /// @brief insert a value in the matrix. If the element already exist, it
    /// overwrites it
    /// @param i row index
    /// @param j column index
    /// @param value value to insert
    /// @throw std::out_of_range if the index is out of range
    void insert_or_assign(Id i, Id j, T value);

    /// @brief insert a value in the matrix. If the element already exist, it
    /// overwrites it
    /// @param index index in vectorial form
    /// @param value value to insert
    /// @throw std::out_of_range if the index is out of range
    void insert_or_assign(Id index, T value);

    /// @brief insert a value in the matrix and expand the matrix if necessary.
    /// @param i row index
    /// @param j column index
    /// @param value value to insert
    void insert_and_expand(Id i, Id j, T value);

    /// @brief remove a value from the matrix
    /// @param i row index
    /// @param j column index
    /// @throw std::out_of_range if the index is out of range
    /// @throw std::runtime_error if the element is not found
    void erase(Id i, Id j);

    /// @brief remove a value from the matrix
    /// @param index index in vectorial form
    /// @throw std::out_of_range if the index is out of range
    /// @throw std::runtime_error if the element is not found
    void erase(Id index);

    /// @brief remove a row from the matrix
    /// @param index row index
    /// @throw std::out_of_range if the index is out of range
    void eraseRow(Id index);

    /// @brief remove a column from the matrix
    /// @param index column index
    /// @throw std::out_of_range if the index is out of range
    void eraseColumn(Id index);

    /// @brief set to 0 all the elements in a row
    void emptyRow(Id index);

    /// @brief set to 0 all the elements in a column
    void emptyColumn(Id index);

    /// @brief empty the matrix and set the dimensions to zero
    void clear();

    /// @brief check if the element is non zero
    /// @param i row index
    /// @param j column index
    /// @return true if the element is non zero
    /// @throw std::out_of_range if the index is out of range
    bool contains(Id i, Id j) const;

    /// @brief check if the element is non zero
    /// @param index index in vectorial form
    /// @return true if the element is non zero
    /// @throw std::out_of_range if the index is out of range
    bool contains(Id const index) const;

    /// @brief get the input degree of all nodes
    /// @return a SparseMatrix vector with the input degree of all nodes
    /// @throw std::runtime_error if the matrix is not square
    SparseMatrix<int> getDegreeVector() const;

    /// @brief get the strength of all nodes
    /// @return a SparseMatrix vector with the strength of all nodes
    /// @throw std::runtime_error if the matrix is not square
    SparseMatrix<double> getStrengthVector() const;

    /// @brief get the laplacian matrix
    /// @return the laplacian matrix
    /// @throw std::runtime_error if the matrix is not square
    SparseMatrix<int> getLaplacian() const;

    /// @brief get a row as a row vector
    /// @param index row index
    /// @param keepId if true, the index of the elements in the row will be
    /// the same as the index of the elements in the matrix
    /// @return a row vector if keepId is false, otherwise a matrix with the
    /// same dimensions as the original matrix
    /// @throw std::out_of_range if the index is out of range
    SparseMatrix getRow(Id index, bool keepId = false) const;

    /// @brief get a column as a column vector
    /// @param index column index
    /// @param keepId if true, the index of the elements in the column will
    /// be the same as the index of the elements in the matrix
    /// @return a column vector if keepId is false, otherwise a matrix with
    /// the same dimensions as the original matrix
    /// @throw std::out_of_range if the index is out of range
    SparseMatrix getCol(Id index, bool keepId = false) const;

    /// @brief get a matrix of double with every row normalized to 1
    /// @return a matrix of double
    SparseMatrix<double> getNormRows() const;

    /// @brief get a matrix of double with every column normalized to 1
    /// @return a matrix of double
    SparseMatrix<double> getNormCols() const;

    /// @brief get the number of rows
    /// @return number of rows
    Id getRowDim() const { return _rows; }

    /// @brief get the number of columns
    /// @return number of columns
    Id getColDim() const { return _cols; }

    /// @brief get the number of non zero elements in the matrix
    /// @return number of non zero elements
    Id size() const { return _matrix.size(); };

    /// @brief get the maximum number of elements in the matrix
    /// @return maximum number of elements
    Id max_size() const { return _rows * _cols; }

    /// @brief symmetrize the matrix
    void symmetrize();

    /// @brief reshape the matrix
    void reshape(Id rows, Id cols);

    /// @brief reshape the matrix
    void reshape(Id dim);

    /// @brief return the begin iterator of the matrix
    /// @return the begin iterator
    typename std::unordered_map<Id, T>::const_iterator begin() const {
      return _matrix.begin();
    }

    /// @brief return the end iterator of the matrix
    /// @return the end iterator
    typename std::unordered_map<Id, T>::const_iterator end() const {
      return _matrix.end();
    }

    /// @brief access an element of the matrix
    /// @param i row index
    /// @param j column index
    /// @return the element
    /// @throw std::out_of_range if the index is out of range
    const T& operator()(Id i, Id j) const;

    /// @brief access an element of the matrix
    /// @param i row index
    /// @param j column index
    /// @return the element
    /// @throw std::out_of_range if the index is out of range
    T& operator()(Id i, Id j);

    /// @brief access an element of the matrix
    /// @param index index in vectorial form
    /// @return the element
    /// @throw std::out_of_range if the index is out of range
    const T& operator()(Id index) const;

    /// @brief access an element of the matrix
    /// @param index index in vectorial form
    /// @return the element
    /// @throw std::out_of_range if the index is out of range
    T& operator()(Id index);

    /// @brief sum of two matrices
    /// @param other the other matrix
    /// @return the sum of the two matrices
    /// @throw std::runtime_error if the dimensions do not match
    template <typename U>
    SparseMatrix<T> operator+(const SparseMatrix<U>& other) {
      if (this->_rows != other._rows || this->_cols != other._cols) {
        throw std::runtime_error(Logger::buildExceptionMessage(
            std::format("Dimensions do not match ({}x{} and {}x{})",
                        this->_rows,
                        this->_cols,
                        other._rows,
                        other._cols)));
      }
      auto result = SparseMatrix<T>(this->_rows, this->_cols);
      std::unordered_map<Id, bool> unique;
      for (auto& it : this->_matrix) {
        unique.insert_or_assign(it.first, true);
      }
      for (auto& it : other._matrix) {
        unique.insert_or_assign(it.first, true);
      }
      for (auto& it : unique) {
        result.insert(it.first / this->_cols,
                      it.first % this->_cols,
                      (*this)(it.first) + other(it.first));
      }
      return result;
    }

    /// @brief difference of two matrices
    /// @param other the other matrix
    /// @return the difference of the two matrices
    /// @throw std::runtime_error if the dimensions do not match
    template <typename U>
    SparseMatrix<T> operator-(const SparseMatrix<U>& other) {
      if (this->_rows != other._rows || this->_cols != other._cols) {
        throw std::runtime_error(Logger::buildExceptionMessage(
            std::format("Dimensions do not match ({}x{} and {}x{})",
                        this->_rows,
                        this->_cols,
                        other._rows,
                        other._cols)));
      }
      auto result = SparseMatrix(this->_rows, this->_cols);
      std::unordered_map<Id, bool> unique;
      for (auto& it : this->_matrix) {
        unique.insert_or_assign(it.first, true);
      }
      for (auto& it : other._matrix) {
        unique.insert_or_assign(it.first, true);
      }
      for (auto& it : unique) {
        result.insert(it.first / this->_cols,
                      it.first % this->_cols,
                      (*this)(it.first) - other(it.first));
      }
      return result;
    }

    /// @brief transpose the matrix
    /// @return the transposed matrix
    SparseMatrix operator++();

    /// @brief sum of two matrices
    /// @param other the other matrix
    /// @return the sum of the two matrices
    /// @throw std::runtime_error if the dimensions do not match
    template <typename U>
    SparseMatrix& operator+=(const SparseMatrix<U>& other);

    /// @brief difference of two matrices
    /// @param other the other matrix
    /// @return the difference of the two matrices
    /// @throw std::runtime_error if the dimensions do not match
    template <typename U>
    SparseMatrix& operator-=(const SparseMatrix<U>& other);
  };

  template <typename T>
  SparseMatrix<T>::SparseMatrix()
      : _matrix{std::unordered_map<Id, T>()}, _rows{}, _cols{}, _defaultReturn{0} {}

  template <typename T>
  SparseMatrix<T>::SparseMatrix(Id rows, Id cols)
      : _matrix{std::unordered_map<Id, T>()},
        _rows{rows},
        _cols{cols},
        _defaultReturn{0} {}

  template <typename T>
  SparseMatrix<T>::SparseMatrix(Id index)
      : _matrix{std::unordered_map<Id, T>()}, _rows{index}, _cols{1}, _defaultReturn{0} {}

  template <typename T>
  void SparseMatrix<T>::insert(Id i, Id j, T value) {
    const auto index = i * _cols + j;
    this->insert(index, value);
  }

  template <typename T>
  void SparseMatrix<T>::insert(Id i, T value) {
    if (i > _rows * _cols - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", i, _rows * _cols - 1)));
    }
    _matrix.emplace(std::make_pair(i, value));
  }

  template <typename T>
  void SparseMatrix<T>::insert_or_assign(Id i, Id j, T value) {
    const auto index = i * _cols + j;
    this->insert_or_assign(index, value);
  }

  template <typename T>
  void SparseMatrix<T>::insert_or_assign(Id index, T value) {
    if (index > _rows * _cols - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows * _cols - 1)));
    }
    _matrix.insert_or_assign(index, value);
  }

  template <typename T>
  void SparseMatrix<T>::insert_and_expand(Id i, Id j, T value) {
    if (!(i < _rows) || !(j < _cols)) {
      Id delta = std::max(i - _rows, j - _cols);
      if (_cols == 1) {
        if (!((i + delta) < (_rows + delta))) {
          ++delta;
        }
        this->reshape(_rows + delta);
      } else {
        if (!((i * (_cols + delta) + j) < (_rows + delta) * (_cols + delta))) {
          ++delta;
        }
        this->reshape(_rows + delta, _cols + delta);
      }
    }
    _matrix.insert_or_assign(i * _cols + j, value);
  }

  template <typename T>
  void SparseMatrix<T>::erase(Id i, Id j) {
    if (i >= _rows || j >= _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id ({}, {}) out of range ({}, {})", i, j, _rows, _cols)));
    }
    if (_matrix.find(i * _cols + j) == _matrix.end()) {
      throw std::runtime_error(Logger::buildExceptionMessage(
          std::format("Element with index {} not found", i * _cols + j)));
    }
    _matrix.erase(i * _cols + j);
  }

  template <typename T>
  void SparseMatrix<T>::erase(Id index) {
    if (index > _rows * _cols - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows * _cols - 1)));
    }
    if (_matrix.find(index) == _matrix.end()) {
      throw std::runtime_error(Logger::buildExceptionMessage(
          std::format("Element with index {} not found", index)));
    }
    _matrix.erase(index);
  }

  template <typename T>
  void SparseMatrix<T>::eraseRow(Id index) {
    if (index > _rows - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows - 1)));
    }
    for (Id i = 0; i < _cols; ++i) {
      _matrix.erase(index * _cols + i);
    }
    std::unordered_map<Id, T> new_matrix = {};
    for (auto const& [key, value] : _matrix) {
      if (key / _cols < index) {
        new_matrix.emplace(std::make_pair(key, value));
      } else {
        new_matrix.emplace(std::make_pair(key - _cols, value));
      }
    }
    --_rows;
    _matrix = new_matrix;
  }

  template <typename T>
  void SparseMatrix<T>::eraseColumn(Id index) {
    if (index > _cols - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _cols - 1)));
    }
    for (Id i = 0; i < _rows; ++i) {
      _matrix.erase(i * _cols + index);
    }
    std::unordered_map<Id, T> new_matrix = {};
    for (auto const& [key, value] : _matrix) {
      if (key % _cols < index) {
        new_matrix.emplace(std::make_pair(key - key / _cols, value));
      } else {
        new_matrix.emplace(
            std::make_pair(key / _cols * (_cols - 1) + key % _cols - 1, value));
      }
    }
    --_cols;
    _matrix = new_matrix;
  }

  template <typename T>
  void SparseMatrix<T>::emptyRow(Id index) {
    for (const auto& x : this->getRow(index)) {
      _matrix.erase(index * _cols + x.first);
    }
  }

  template <typename T>
  void SparseMatrix<T>::emptyColumn(Id index) {
    for (const auto& x : this->getCol(index)) {
      _matrix.erase(x.first * _cols + index);
    }
  }

  template <typename T>
  void SparseMatrix<T>::clear() {
    _matrix.clear();
    _rows = 0;
    _cols = 0;
  }

  template <typename T>
  bool SparseMatrix<T>::contains(Id i, Id j) const {
    if (i >= _rows || j >= _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id ({}, {}) out of range ({}, {})", i, j, _rows, _cols)));
    }
    return _matrix.contains(i * _cols + j);
  }

  template <typename T>
  bool SparseMatrix<T>::contains(Id const index) const {
    if (index > _rows * _cols - 1) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows * _cols - 1)));
    }
    return _matrix.contains(index);
  }

  template <typename T>
  SparseMatrix<int> SparseMatrix<T>::getDegreeVector() const {
    if (_rows != _cols) {
      throw std::runtime_error(
          Logger::buildExceptionMessage("getDegreeVector only works on square matrices"));
    }
    auto degreeVector = SparseMatrix<int>(_rows, 1);
    for (auto& i : _matrix) {
      degreeVector.insert_or_assign(
          i.first / _cols, 0, degreeVector(i.first / _cols, 0) + 1);
    }
    return degreeVector;
  }

  template <typename T>
  SparseMatrix<double> SparseMatrix<T>::getStrengthVector() const {
    if (_rows != _cols) {
      throw std::runtime_error(Logger::buildExceptionMessage(
          "getStrengthVector only works on square matrices"));
    }
    auto strengthVector = SparseMatrix<double>(_rows, 1);
    for (auto& i : _matrix) {
      strengthVector.insert_or_assign(
          i.first / _cols, 0, strengthVector(i.first / _cols, 0) + i.second);
    }
    return strengthVector;
  }

  template <typename T>
  SparseMatrix<int> SparseMatrix<T>::getLaplacian() const {
    if (_rows != _cols) {
      throw std::runtime_error(
          Logger::buildExceptionMessage("getLaplacian only works on square matrices"));
    }
    auto laplacian = SparseMatrix<int>(_rows, _cols);
    for (auto& i : _matrix) {
      laplacian.insert_or_assign(i.first / _cols, i.first % _cols, -1);
    }
    auto degreeVector = this->getDegreeVector();
    for (Id i = 0; i < _rows; ++i) {
      laplacian.insert_or_assign(i, i, degreeVector(i, 0));
    }
    return laplacian;
  }

  template <typename T>
  SparseMatrix<T> SparseMatrix<T>::getRow(Id index, bool keepId) const {
    if (index >= _rows) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows - 1)));
    }
    SparseMatrix row(1, _cols);
    if (keepId) {
      row.reshape(_rows, _cols);
    }
    for (auto& it : _matrix) {
      if (it.first / _cols == index) {
        keepId ? row.insert(it.first, it.second)
               : row.insert(it.first % _cols, it.second);
      }
    }
    return row;
  }

  template <typename T>
  SparseMatrix<T> SparseMatrix<T>::getCol(Id index, bool keepId) const {
    if (index >= _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _cols - 1)));
    }
    SparseMatrix col(_rows, 1);
    if (keepId) {
      col.reshape(_rows, _cols);
    }
    for (auto& it : _matrix) {
      if (it.first % _cols == index) {
        keepId ? col.insert(it.first, it.second)
               : col.insert(it.first / _cols, it.second);
      }
    }
    return col;
  }

  template <typename T>
  SparseMatrix<double> SparseMatrix<T>::getNormRows() const {
    SparseMatrix<double> normRows(_rows, _cols);
    for (Id index = 0; index < _rows; ++index) {
      auto row = this->getRow(index, true);
      double sum = 0.;
      for (auto& it : row) {
        sum += std::abs(it.second);
      }
      if (sum < std::numeric_limits<double>::epsilon()) {
        sum = 1.;
      }
      for (auto& it : row) {
        normRows.insert(it.first, it.second / sum);
      }
    }
    return normRows;
  }

  template <typename T>
  SparseMatrix<double> SparseMatrix<T>::getNormCols() const {
    SparseMatrix<double> normCols(_rows, _cols);
    for (Id index = 0; index < _cols; ++index) {
      auto col = this->getCol(index, true);
      double sum = 0.;
      for (auto& it : col) {
        sum += std::abs(it.second);
      }
      if (sum < std::numeric_limits<double>::epsilon()) {
        sum = 1.;
      }
      for (auto& it : col) {
        normCols.insert(it.first, it.second / sum);
      }
    }
    return normCols;
  }

  template <typename T>
  void SparseMatrix<T>::symmetrize() {
    *this += this->operator++();
  }

  template <typename T>
  void SparseMatrix<T>::reshape(Id rows, Id cols) {
    Id oldCols = this->_cols;
    this->_rows = rows;
    this->_cols = cols;
    auto copy = _matrix;
    for (auto& it : copy) {
      _matrix.erase(it.first);
      if (it.first < rows * cols) {
        this->insert_or_assign(it.first / oldCols, it.first % oldCols, it.second);
      }
    }
  }

  template <typename T>
  void SparseMatrix<T>::reshape(Id index) {
    this->_rows = index;
    this->_cols = 1;
    auto copy = _matrix;
    for (auto& it : copy) {
      _matrix.erase(it.first);
      if (it.first < index) {
        this->insert_or_assign(it.first, it.second);
      }
    }
  }

  template <typename T>
  const T& SparseMatrix<T>::operator()(Id i, Id j) const {
    if (i >= _rows || j >= _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id ({}, {}) out of range ({}, {})", i, j, _rows, _cols)));
    }
    auto const& it = _matrix.find(i * _cols + j);
    return it != _matrix.end() ? it->second : _defaultReturn;
  }

  template <typename T>
  T& SparseMatrix<T>::operator()(Id i, Id j) {
    if (i >= _rows || j >= _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id ({}, {}) out of range ({}, {})", i, j, _rows, _cols)));
    }
    auto const& it = _matrix.find(i * _cols + j);
    return it != _matrix.end() ? it->second : _defaultReturn;
  }

  template <typename T>
  const T& SparseMatrix<T>::operator()(Id index) const {
    if (index >= _rows * _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows * _cols - 1)));
    }
    auto const& it = _matrix.find(index);
    return it != _matrix.end() ? it->second : _defaultReturn;
  }

  template <typename T>
  T& SparseMatrix<T>::operator()(Id index) {
    if (index >= _rows * _cols) {
      throw std::out_of_range(Logger::buildExceptionMessage(
          std::format("Id {} out of range 0-{}", index, _rows * _cols - 1)));
    }
    auto const& it = _matrix.find(index);
    return it != _matrix.end() ? it->second : _defaultReturn;
  }

  template <typename T>
  SparseMatrix<T> SparseMatrix<T>::operator++() {
    auto transpost = SparseMatrix(this->_cols, this->_rows);
    for (auto& it : _matrix) {
      transpost.insert(it.first % _cols, it.first / _cols, it.second);
    }
    return transpost;
  }

  template <typename T>
  template <typename U>
  SparseMatrix<T>& SparseMatrix<T>::operator+=(const SparseMatrix<U>& other) {
    if (this->_rows != other._rows || this->_cols != other._cols) {
      Logger::error(std::format("Dimensions do not match ({}x{} and {}x{})",
                                this->_rows,
                                this->_cols,
                                other._rows,
                                other._cols));
    }
    for (auto& it : other._matrix) {
      this->contains(it.first)
          ? this->insert_or_assign(it.first, this->operator()(it.first) + it.second)
          : this->insert(it.first, it.second);
    }
    return *this;
  }

  template <typename T>
  template <typename U>
  SparseMatrix<T>& SparseMatrix<T>::operator-=(const SparseMatrix<U>& other) {
    if (this->_rows != other._rows || this->_cols != other._cols) {
      Logger::error(std::format("Dimensions ({}, {}) and ({}, {}) do not match",
                                this->_rows,
                                this->_cols,
                                other._rows,
                                other._cols));
    }
    for (auto& it : other._matrix) {
      this->contains(it.first)
          ? this->insert_or_assign(it.first, this->operator()(it.first) - it.second)
          : this->insert(it.first, -it.second);
    }
    return *this;
  }
};  // namespace dsm
