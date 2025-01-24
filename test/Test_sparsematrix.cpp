
#include "SparseMatrix.hpp"

#include "doctest.h"

using namespace dsm;

TEST_CASE("Boolean Matrix") {
  SUBCASE("Default constructor") {
    /*This test tests if the default constructor works correctly
    The default constructor should create a matrix with 0 rows and 0 columns
    and a max size of 0
    GIVEN: the default constructor is called
    WHEN: the matrix is created
    THEN: the matrix should have 0 rows and 0 columns and a max size of 0
    */
    SparseMatrix<bool> m;
    // Check the dimensions
    CHECK(m.getRowDim() == 0);
    CHECK(m.getColDim() == 0);
    CHECK(m.max_size() == 0);
  }
  SUBCASE("Constructor with dimensions") {
    /*This test tests if the constructor with dimensions works correctly
    The constructor should create a matrix with the specified dimensions and a
    max size equal to the product of the dimensions
    GIVEN: the constructor is called with dimensions 3 and 4
    WHEN: the matrix is created
    THEN: the matrix should have 3 rows and 4 columns and a max size of 12
    */
    SparseMatrix<bool> m(3, 4);
    // Check the dimensions
    CHECK(m.getRowDim() == 3);
    CHECK(m.getColDim() == 4);
    CHECK(m.max_size() == 12);
  }
  SUBCASE("Constructor with dimension") {
    /*This test tests if the constructor with dimension works correctly
    The constructor should create a row vector with the specified dimension
    GIVEN: the constructor is called with dimension 3
    WHEN: the matrix is created
    THEN: the matrix should have 3 rows and 1 column and a max size of 3
    */
    SparseMatrix<bool> m(3);
    // Check the dimensions
    CHECK(m.getRowDim() == 3);
    CHECK(m.getColDim() == 1);
    CHECK(m.max_size() == 3);
  }
  SUBCASE("insert_or_assign exceptions") {
    /*This test tests if the insert_or_assign function throws exceptions
    correctly The insert_or_assign function should throw an exception if the
    inserted element is out of range
    GIVEN: the insert_or_assign function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the inserted element is out
    of range
    */
    SparseMatrix<bool> m(3, 3);
    // Check that an exception is thrown if the element is out of range
    CHECK_THROWS(m.insert_or_assign(3, 2, true));
  }
  SUBCASE("Insertions") {
    /*This test tests if the insert function works correctly
    The insert function should insert a value in the matrix
    GIVEN: the insert function is called
    WHEN: the function is called on a matrix
    THEN: the function should insert a value in the matrix
    */
    SparseMatrix<bool> m(3, 3);
    // Insert a true value
    m.insert(0, 0, true);
    m.insert(5, true);
    // Check all values
    CHECK(m(0, 0));
    CHECK(m(1, 2));
    for (uint32_t i = 1; i < 9; ++i) {
      if (i != 5) {
        CHECK(!m(i / 3, i % 3));
      }
    }
  }
  SUBCASE("insert_or_assign") {
    /*This test tests if the insert_or_assign function works correctly
    The insert_or_assign function should insert a value in the matrix
    or assign a new value to an existing element
    GIVEN: the insert_or_assign function is called
    WHEN: the function is called on a matrix
    THEN: the function should insert a value in the matrix
    */
    SparseMatrix<int> m(4, 3);
    // Insert a true value
    m.insert_or_assign(1, 2, 10);
    CHECK(m(1, 2) == 10);
    m.insert_or_assign(1, 2, 20);
    CHECK(m(1, 2) == 20);
  }
  SUBCASE("erase - exception") {
    /*This test tests if the erase function throws an exception correctly
    The erase function should throw an exception if the element is out of range
    GIVEN: the erase function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the element is out of range
    */
    SparseMatrix<bool> m(3, 3);
    // Check that an exception is thrown if the element is out of range
    CHECK_THROWS(m.erase(-1, -2));
    CHECK_THROWS(m.erase(3, 2));
  }
  SUBCASE("Deletions") {
    /*This test tests if the erase function works correctly
    The erase function should delete a value in the matrix
    GIVEN: the erase function is called
    WHEN: the function is called on a matrix
    THEN: the function should delete a value in the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.erase(0, 0);
    // Check if the value has been deleted
    CHECK(!m(0, 0));
  }
  SUBCASE("Clear") {
    /*This test tests if the clear function works correctly
    The clear function should delete all the elements and dimensions in the
    matrix
    GIVEN: the clear function is called
    WHEN: the function is called on a matrix
    THEN: the function should delete all the elements and dimensions in the
    matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.clear();
    // Check if the matrix is empty
    CHECK(m.size() == 0);
    CHECK_THROWS(m(0, 0));
  }
  SUBCASE("Contains exceptions") {
    /*This test tests if the contains function throws exceptions correctly
    The contains function should throw an exception if the element is out of
    range
    GIVEN: the contains function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the element is out of range
    */
    SparseMatrix<bool> m(3, 3);
    CHECK_THROWS(m.contains(-2, -1));
    CHECK_THROWS(m.contains(-1));
  }
  SUBCASE("Contains") {
    /*This test tests if the contains function works correctly
    The contains function should return true if the matrix contains the element
    GIVEN: the element is in the matrix
    WHEN: the function is called
    THEN: the function should return true
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(2, 1, true);
    CHECK(m.contains(0, 0));
    CHECK(m.contains(7));
  }
  SUBCASE("getRow - exceptions") {
    /*This test tests if the getRow function throws exceptions correctly
    The getRow function should throw an exception if the row is out of range
    GIVEN: the getRow function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the row is out of range
    */
    SparseMatrix<bool> m(4, 3);
    CHECK_THROWS(m.getRow(-1));
    CHECK_THROWS(m.getRow(4));
  }
  SUBCASE("getRow") {
    /*This test tests if the getRow function works correctly
    The getRow function should return a vector containing the elements of the
    row
    GIVEN: the getRow function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a vector (SparseMatrix) containing the
    elements of the row
    */
    SparseMatrix<bool> m(3, 3);
    // Create a row
    m.insert(0, 0, true);
    m.insert(0, 2, true);
    auto row = m.getRow(0);
    // verify attributes and values
    CHECK(row.size() == 2);
    CHECK(row(0));
    CHECK(!row(1));
    CHECK(row(2));
  }
  SUBCASE("getRow with keepIndex true") {
    /*This test tests if the getRow function works correctly
    The getRow function should return a vector containing the elements of the
    row
    GIVEN: the getRow function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a vector (SparseMatrix) containing the
    elements of the row
    */
    SparseMatrix<bool> m(3, 3);
    // Create a row
    m.insert(0, 0, true);
    m.insert(0, 2, true);
    auto row = m.getRow(0, true);
    // verify attributes and values
    CHECK(row.size() == 2);
    CHECK(row(0));
    CHECK(!row(1));
    CHECK(row(2));
  }
  SUBCASE("getColumn - exceptions") {
    /*This test tests if the getColumn function throws exceptions correctly
    The getColumn function should throw an exception if the column is out of
    range
    GIVEN: the getColumn function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the column is out of range
    */
    SparseMatrix<bool> m(3, 6);
    CHECK_THROWS(m.getCol(6));
  }
  SUBCASE("getCol") {
    /*This test tests if the getCol function works correctly
    The getCol function should return a vector containing the elements of the
    column
    GIVEN: the getCol function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a vector (SparseMatrix) containingthe
    elements of the column
    */
    SparseMatrix<bool> m(3, 3);
    // Create a column
    m.insert(0, 0, true);
    m.insert(2, 0, true);
    auto col = m.getCol(0);
    // verify attributes and values
    CHECK(col.size() == 2);
    CHECK(col(0));
    CHECK(!col(1));
    CHECK(col(2));
  }
  SUBCASE("getCol with keepIndex true") {
    /*This test tests if the getCol function works correctly
    The getCol function should return a vector containing the elements of the
    column
    GIVEN: the getCol function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a vector (SparseMatrix) containingthe
    elements of the column
    */
    SparseMatrix<bool> m(3, 3);
    // Create a column
    m.insert(0, 0, true);
    m.insert(2, 0, true);
    auto col = m.getCol(0, true);
    // verify attributes and values
    CHECK(col.size() == 2);
    CHECK(col(0));
    CHECK(!col(1));
    CHECK(col(6));
  }
  SUBCASE("Get row dimension") {
    /*This test tests if the getRowDim function works correctly
    The getRowDim function should return the number of rows in the matrix
    GIVEN: the getRowDim function is called
    WHEN: the function is called on a matrix
    THEN: the function should return the number of rows in the matrix
    */
    SparseMatrix<bool> m(7, 3);
    CHECK(m.getRowDim() == 7);
  }
  SUBCASE("Get column dimension") {
    /*This test tests if the getColDim function works correctly
    The getColDim function should return the number of columns in the matrix
    GIVEN: the getColDim function is called
    WHEN: the function is called on a matrix
    THEN: the function should return the number of columns in the matrix
    */
    SparseMatrix<bool> m(3, 10);
    CHECK(m.getColDim() == 10);
  }
  SUBCASE("Get max_size") {
    /*This test tests if the max_size function works correctly
    The max_size function should return the maximum number of elements that can
    be stored in the matrix
    GIVEN: the max_size function is called
    WHEN: the function is called on a matrix
    THEN: the function should return the maximum number of elements that can be
    stored in the matrix
    */
    SparseMatrix<bool> m(3, 5);
    CHECK(m.max_size() == 15);
  }
  SUBCASE("Get size") {
    /*This test tests if the size function works correctly
    The size function should return the number of non-zero elements in the
    matrix
    GIVEN: the size function is called
    WHEN: the function is called on a matrix
    THEN: the function should return the number of elements in the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(0, 2, true);
    CHECK(m.size() == 3);
    m.insert(1, 1, true);
    CHECK(m.size() == 4);
  }
  SUBCASE("eraseRow - exception") {
    /*This test tests if the eraseRow function throws an exception
    The eraseRow function should throw an exception if the row is out of range
    GIVEN: the eraseRow function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the row is out of range
    */
    SparseMatrix<bool> m(5, 3);
    CHECK_THROWS(m.eraseRow(5));
  }
  SUBCASE("Erase row") {
    /*This test tests if the eraseRow function works correctly
    The eraseRow function should delete all the elements in the row (and also
    the row itself, reducing the number of rows by 1)
    GIVEN: the eraseRow function is called
    WHEN: the function is called on a matrix
    THEN: the function should delete all the elements in the row
    */
    SparseMatrix<bool> d(3, 3);
    // Create a row
    d.insert(0, 0, true);
    d.insert(1, 2, true);
    d.insert(2, 1, true);
    // Clone the matrix
    auto m = d;
    // Erase the row (for each row) and check if all the other elements are
    // rearranged correctly
    m.eraseRow(1);
    // Check the values
    CHECK(m(0, 0));
    CHECK(m(1, 1));
    m = d;
    m.eraseRow(0);
    CHECK(m(0, 2));
    CHECK(m(1, 1));
    m = d;
    m.eraseRow(2);
    CHECK(m(0, 0));
    CHECK(m(1, 2));
    m.eraseRow(0);
    CHECK(m(0, 2));
  }
  SUBCASE("eraseColumn - exception") {
    /*This test tests if the eraseColumn function throws an exception
    The eraseColumn function should throw an exception if the column is out of
    range
    GIVEN: the eraseColumn function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the column is out of range
    */
    SparseMatrix<bool> m(3, 5);
    CHECK_THROWS(m.eraseColumn(5));
  }
  SUBCASE("Erase column") {
    /*This test tests if the eraseColumn function works correctly
    The eraseColumn function should delete all the elements in the column (and
    also the column itself, reducing the number of columns by 1)
    GIVEN: the eraseColumn function is called
    WHEN: the function is called on a matrix
    THEN: the function should delete all the elements in the column
    */
    SparseMatrix<bool> d(3, 3);
    d.insert(0, 0, true);
    d.insert(1, 2, true);
    d.insert(2, 1, true);
    auto m = d;
    // Erase the row (for each row) and check if all the other elements are
    // rearranged correctly
    m.eraseColumn(1);
    CHECK(m(0, 0));
    CHECK(m(1, 1));
    m = d;
    m.eraseColumn(0);
    CHECK(m(1, 1));
    CHECK(m(2, 0));
    m = d;
    m.eraseColumn(2);
    CHECK(m(0, 0));
    CHECK(m(2, 1));
  }
  SUBCASE("getDegreeVector - exceptions") {
    /*This test tests if the getDegreeVector function throws an exception
    The getDegreeVector function should throw an exception if the matrix is
    empty
    GIVEN: the getDegreeVector function is called
    WHEN: the function is called on a matrix
    THEN: the function should throw an exception if the matrix is empty
    */
    SparseMatrix<bool> m(1, 5);
    SparseMatrix<bool> m2(3, 6);
    CHECK_THROWS(m.getDegreeVector());
    CHECK_THROWS(m2.getDegreeVector());
  }
  SUBCASE("Degree vector") {
    /*This test tests if the getDegreeVector function works correctly
    The getDegreeVector function should return a vector containing the degree of
    each row
    GIVEN: the getDegreeVector function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a vector containing the degree of each row
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    m.insert(2, 0, true);
    m.insert(2, 1, true);
    m.insert(2, 2, true);
    auto v = m.getDegreeVector();
    // check if the sum on all rows is done correctly
    CHECK(v(0) == 2);
    CHECK(v(1) == 1);
    CHECK(v(2) == 3);
  }
  SUBCASE("Normalized Rows") {
    /*This test tests if the getNormRows function works correctly
    The getNormRows function should return a matrix containing the normalized
    rows
    GIVEN: the getNormRows function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a matrix containing the normalized rows
    */
    SparseMatrix<bool> m(3, 3);
    // Create a row
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(0, 2, true);
    m.insert(1, 1, true);
    m.insert(1, 2, true);
    m.insert(2, 1, true);
    // Get the normalized rows and check their values, comparig floats with a
    // numeric limit
    auto v = m.getNormRows();
    CHECK(v(0, 0) - 1. / 3 < std::numeric_limits<double>::epsilon());
    CHECK(v(1, 1) - 1. / 2 < std::numeric_limits<double>::epsilon());
    CHECK(v(2, 1) - 1 < std::numeric_limits<double>::epsilon());
    CHECK(v(0, 0) + v(0, 1) + v(0, 2) - 1 < std::numeric_limits<double>::epsilon());
    CHECK(v(1, 0) + v(1, 1) + v(1, 2) - 1 < std::numeric_limits<double>::epsilon());
    CHECK(v(2, 0) + v(2, 1) + v(2, 2) - 1 < std::numeric_limits<double>::epsilon());
  }
  SUBCASE("Normalized Columns") {
    /*This test tests if the getNormCols function works correctly
    The getNormCols function should return a matrix containing the normalized
    columns
    GIVEN: the getNormCols function is called
    WHEN: the function is called on a matrix
    THEN: the function should return a matrix containing the normalized columns
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(0, 2, true);
    m.insert(1, 1, true);
    m.insert(1, 2, true);
    m.insert(2, 1, true);
    // Get the normalized columns and check their values, comparig floats with a
    // numeric limit
    auto v = m.getNormCols();
    CHECK(v(0, 0) == doctest::Approx(1.));
    CHECK(v(1, 1) == doctest::Approx(1. / 3));
    CHECK(v(1, 2) == doctest::Approx(1. / 2));
    CHECK(v(0, 0) + v(1, 0) + v(2, 0) == doctest::Approx(1.));
    CHECK(v(0, 1) + v(1, 1) + v(2, 1) == doctest::Approx(1.));
    CHECK(v(0, 2) + v(1, 2) + v(2, 2) == doctest::Approx(1.));
  }
  SUBCASE("Symmetrization") {
    /*This test tests if the symmetrize function works correctly
    The symmetrize function should symmetrize the matrix
    GIVEN: the symmetrize function is called
    WHEN: the function is called on a matrix
    THEN: the function should symmetrize the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    // Symmetrize the matrix
    m.symmetrize();
    CHECK(m(0, 0));
    CHECK(m(0, 1));
    CHECK(m(1, 0));
    CHECK(m(1, 2));
    CHECK(m(2, 1));
    CHECK(m.size() == 5);
  }
  SUBCASE("+ operator - exception") {
    /*This test tests if the + operator throws an exception correctly
    The + operator should throw an exception if the matrices have different
    dimensions
    GIVEN: the + operator is called
    WHEN: the function is called on two matrices
    THEN: the function should throw an exception if the matrices have different
    dimensions
    */
    SparseMatrix<bool> m(3, 3);
    SparseMatrix<bool> m2(3, 4);
    CHECK_THROWS(m + m2);
  }
  SUBCASE("+ operator") {
    /*This test tests if the + operator works correctly
    The + operator should sum two matrices
    GIVEN: the + operator is called
    WHEN: the function is called on two matrices
    THEN: the function should sum the two matrices
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    SparseMatrix<bool> m2(3, 3);
    m2.insert(0, 0, true);
    m2.insert(1, 0, true);
    m2.insert(2, 1, true);
    auto m3 = m + m2;
    CHECK(m3(0, 0));
    CHECK(m3(1, 0));
    CHECK(m3(2, 1));
    CHECK(m3(0, 1));
    CHECK(m3(1, 2));
    CHECK(m3.size() == 5);
  }
  SUBCASE("+= operator") {
    /*This test tests if the += operator works correctly
    The += operator should sum two matrices
    GIVEN: the += operator is called
    WHEN: the function is called on two matrices
    THEN: the function should sum the two matrices
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    SparseMatrix<bool> m2(3, 3);
    m2.insert(0, 0, true);
    m2.insert(1, 0, true);
    m2.insert(2, 1, true);
    m += m2;
    CHECK(m(0, 0));
    CHECK(m(1, 0));
    CHECK(m(2, 1));
    CHECK(m(0, 1));
    CHECK(m(1, 2));
    CHECK(m.size() == 5);
  }
  SUBCASE("- operator - exception") {
    /*This test tests if the - operator throws an exception correctly
    The - operator should throw an exception if the matrices have different
    dimensions
    GIVEN: the - operator is called
    WHEN: the function is called on two matrices
    THEN: the function should throw an exception if the matrices have different
    dimensions
    */
    SparseMatrix<bool> m(3, 3);
    SparseMatrix<bool> m2(3, 4);
    CHECK_THROWS(m - m2);
  }
  SUBCASE("- operator") {
    /*This test tests if the - operator works correctly
    The - operator should subtract two matrices
    GIVEN: the - operator is called
    WHEN: the function is called on two matrices
    THEN: the function should subtract the two matrices
    */
    SparseMatrix<int> m(3, 3);
    m.insert(0, 0, 1);
    m.insert(0, 1, 2);
    m.insert(1, 2, 3);
    SparseMatrix<int> m2(3, 3);
    m2.insert(0, 0, 1);
    m2.insert(1, 0, 2);
    m2.insert(2, 1, 3);
    auto m3 = m - m2;
    CHECK(m3(0, 0) == 0);
    CHECK(m3(1, 0) == -2);
    CHECK(m3(2, 1) == -3);
    CHECK(m3(0, 1) == 2);
    CHECK(m3(1, 2) == 3);
    CHECK(m3.size() == 5);
  }
  SUBCASE("-= operator") {
    /*This test tests if the -= operator works correctly
    The -= operator should subtract two matrices
    GIVEN: the -= operator is called
    WHEN: the function is called on two matrices
    THEN: the function should subtract the two matrices
    */
    SparseMatrix<int> m(3, 3);
    m.insert(0, 0, 1);
    m.insert(0, 1, 2);
    m.insert(1, 2, 3);
    SparseMatrix<int> m2(3, 3);
    m2.insert(0, 0, 1);
    m2.insert(1, 0, 2);
    m2.insert(2, 1, 3);
    m -= m2;
    CHECK(m(0, 0) == 0);
    CHECK(m(1, 0) == -2);
    CHECK(m(2, 1) == -3);
    CHECK(m(0, 1) == 2);
    CHECK(m(1, 2) == 3);
    CHECK(m.size() == 5);
  }
  SUBCASE("getStrengthVector - exception") {
    /*This test tests if the getStrengthVector function throws an exception
    The getStrengthVector function should throw an exception if the matrix is
    not square
    GIVEN: the getStrengthVector function is called
    WHEN: the function is called on a non-square matrix
    THEN: the function should throw an exception
    */
    SparseMatrix<double> m(4, 3);
    SparseMatrix<double> m2(3, 4);
    CHECK_THROWS(m.getStrengthVector());
    CHECK_THROWS(m2.getStrengthVector());
  }
  SUBCASE("getStrengthVector") {
    /*This test tests if the getStrengthVector function works correctly
    The getStrengthVector function should return the strength vector of the
    matrix
    GIVEN: the getStrengthVector function is called
    WHEN: the function is called on a street with 1 vehicle and length 10
    THEN: the function should return a vector with 0.1
    */
    SparseMatrix<double> m(3, 3);
    m.insert(0, 0, 0.3);
    m.insert(0, 1, 0.3);
    m.insert(0, 2, 0.1);
    m.insert(1, 1, 0.5);
    m.insert(2, 0, 0.1);
    m.insert(2, 2, 1.);
    auto v = m.getStrengthVector();
    CHECK(v(0) == doctest::Approx(0.7));
    CHECK(v(1) == doctest::Approx(0.5));
    CHECK(v(2) == doctest::Approx(1.1));
    CHECK(v.size() == 3);
  }
  SUBCASE("getLaplacian - exception") {
    /*This test tests if the getLaplacian function throws an exception
    The getLaplacian function should throw an exception if the street is empty
    GIVEN: the getLaplacian function is called
    WHEN: the function is called on a non-square matrix
    THEN: the function should throw an exception
    */
    SparseMatrix<double> m(4, 3);
    CHECK_THROWS(m.getLaplacian());
  }
  SUBCASE("getLaplacian") {
    /*This test tests if the getLaplacian function works correctly
    The getLaplacian function should return the Laplacian matrix of the matrix
    GIVEN: the getLaplacian function is called
    WHEN: the function is called on a matrix
    THEN: the function should return the Laplacian matrix of the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    auto m2 = m.getLaplacian();
    CHECK(m2(0, 0) == 2);
    CHECK(m2(0, 1) == -1);
    CHECK(m2(1, 1) == 1);
    CHECK(m2(1, 2) == -1);
    CHECK(m2(2, 2) == 0);
    CHECK(m2.size() == 5);
  }
  SUBCASE("Reshape rectangular") {
    /*This test tests if the reshape function works correctly
    The reshape function should reshape the matrix
    GIVEN: the reshape function is called
    WHEN: the function is called on a matrix
    THEN: the function should reshape the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    m.reshape(2, 3);
    CHECK(m(0, 0));
    CHECK(m(0, 1));
    CHECK(m(1, 2));
    CHECK(m.size() == 3);
  }
  SUBCASE("Reshape into a column vector") {
    /*This test tests if the reshape function works correctly
    The reshape function should reshape the matrix
    GIVEN: the reshape function is called
    WHEN: the function is called on a matrix
    THEN: the function should reshape the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    m.reshape(2);
    CHECK(m(0));
    CHECK(m(1));
    CHECK_THROWS(m(1, 2));
    CHECK(m.size() == 2);
  }
  SUBCASE("reshape in greater dimension") {
    /*
    The reshape function should reshape the matrix
    GIVEN: the reshape function is called
    WHEN: the function is called on a matrix
    THEN: the function should reshape the matrix
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(0, 1, true);
    m.insert(1, 2, true);
    m.reshape(4, 4);
    CHECK(m(0, 0));
    CHECK(m(0, 1));
    CHECK(m(1, 2));
  }
  SUBCASE("insert_and_expand") {
    /*
    The insert_and_expand function should insert a value in the matrix
    and expand the matrix if necessary
    GIVEN: the insert_and_expand function is called
    WHEN: the function is called on a matrix
    THEN: the function should insert a value in the matrix and expanding it
    */
    SparseMatrix<bool> m(3, 3);
    m.insert(0, 0, true);
    m.insert(1, 2, true);
    m.insert_and_expand(3, 4, true);
    CHECK(m(0, 0));
    CHECK(m(1, 2));
    CHECK(m(3, 4));
    CHECK(m.size() == 3);
    CHECK(m.getRowDim() == 5);
    CHECK(m.getColDim() == 5);
  }
}
