/// @file       /src/dsf/headers/AdjacencyMatrix.hpp
/// @brief      Defines the AdjacencyMatrix class.

#pragma once

#include <algorithm>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "../utility/Typedef.hpp"

namespace dsf {

  class AdjacencyMatrix;

  namespace test {
    std::vector<Id> offsets(const AdjacencyMatrix& adj);
    std::vector<Id> indices(const AdjacencyMatrix& adj);
  }  // namespace test

  /// @brief The AdjacencyMatrix class represents the adjacency matrix of the network.
  /// @details The AdjacencyMatrix class represents the adjacency matrix of the network.
  ///          It is defined as \f$A = (a_{ij})\f$, where \f$a_{ij} \in \{0, 1\}\f$.
  ///          Moreover, \f$a_{ij} = 1\f$ if there is an edge from node \f$i\f$ to node \f$j\f$ and \f$a_{ij} = 0\f$ otherwise.
  ///          It is used to store the adjacency matrix of the network and to perform operations on it.
  ///          The adjacency matrix is stored in both CSR and CSC formats, to optimize access to rows and columns.
  ///          Thus, this matrix has very fast access, using double the memory of a standard CSR/CSC one.
  class AdjacencyMatrix {
  private:
    // CSR format
    std::vector<Id> m_rowOffsets;
    std::vector<Id> m_columnIndices;
    // CSC format
    std::vector<Id> m_colOffsets;
    std::vector<Id> m_rowIndices;
    // Size of the matrix
    size_t m_n;

    friend std::vector<Id> test::offsets(const AdjacencyMatrix& adj);
    friend std::vector<Id> test::indices(const AdjacencyMatrix& adj);

  public:
    /// @brief Construct a new AdjacencyMatrix object
    AdjacencyMatrix();
    /// @brief Construct a new AdjacencyMatrix object using the @ref read method
    /// @param fileName The name of the file containing the adjacency matrix
    AdjacencyMatrix(std::string const& fileName);

    bool operator==(const AdjacencyMatrix& other) const;
    /// @brief Get the link at the specified row and column
    /// @param row The row index of the element
    /// @param col The column index of the element
    /// @return True if the link exists, false otherwise
    /// @details This function actually returns element \f$a_{ij}\f$ of the adjacency matrix.
    ///   Where \f$i\f$ is the row index and \f$j\f$ is the column index.
    bool operator()(Id row, Id col) const;

    /// @brief Get the number of links in the adjacency matrix
    /// @return The number of links in the adjacency matrix
    size_t size() const;
    /// @brief Check if the adjacency matrix is empty
    /// @return True if the adjacency matrix is empty, false otherwise
    bool empty() const;
    /// @brief Get the number of nodes in the adjacency matrix
    /// @return The number of nodes in the adjacency matrix
    size_t n() const;
    /// @brief Inserts the link row -> col in the adjacency matrix
    /// @param row The row index of the element
    /// @param col The column index of the element
    /// @details This function inserts the link \f$(row, col)\f$ in the adjacency matrix.
    ///   Where \f$row\f$ is the row index and \f$col\f$ is the column index.
    void insert(Id row, Id col);
    /// @brief Check if the link row -> col exists in the adjacency matrix
    /// @param row The row index of the element
    /// @param col The column index of the element
    /// @return True if the link exists, false otherwise
    /// @details This function actually returns element \f$a_{ij}\f$ of the adjacency matrix.
    ///   Where \f$i\f$ is the row index and \f$j\f$ is the column index.
    bool contains(Id row, Id col) const;
    /// @brief Get the row at the specified index
    /// @param row The row index
    /// @return The row at the specified index
    std::vector<Id> getRow(Id row) const;
    /// @brief Get the column at the specified index
    /// @param col The column index
    /// @return The column at the specified index
    std::vector<Id> getCol(Id col) const;
    /// @brief Get a vector containing all the links in the adjacency matrix as pairs of nodes
    /// @return A vector containing all the links in the adjacency matrix as pairs of nodes
    std::vector<std::pair<Id, Id>> elements() const;

    /// @brief Clear the adjacency matrix
    void clear();
    /// @brief Clear the row at the specified index
    /// @details The dimension of the matrix does not change.
    void clearRow(Id row);
    /// @brief Clear the column at the specified index
    /// @details The dimension of the matrix does not change.
    void clearCol(Id col);

    /// @brief Get the input degree vector of the adjacency matrix
    /// @return The input degree vector of the adjacency matrix
    std::vector<int> getInDegreeVector() const;
    /// @brief Get the output degree vector of the adjacency matrix
    /// @return The output degree vector of the adjacency matrix
    std::vector<int> getOutDegreeVector() const;

    /// @brief Read the adjacency matrix from a binary file
    /// @param fileName The name of the file containing the adjacency matrix
    void read(std::string const& fileName);
    /// @brief Write the adjacency matrix to a binary file
    /// @param fileName The name of the file where the adjacency matrix will be written
    void save(std::string const& fileName) const;
  };
}  // namespace dsf
