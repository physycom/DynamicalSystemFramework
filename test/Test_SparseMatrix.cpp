#include "SparseMatrix.hpp"

#include "doctest.h"

using namespace dsm;

TEST_CASE("Double matrix") {
  using Matrix = SparseMatrix<double>;
  SUBCASE("Default constructor and insertion") {
    Matrix mat;
    CHECK(mat.empty());
    CHECK_EQ(mat.size(), 0);
    CHECK_EQ(mat.n(), 1);
    CHECK_EQ(mat(0, 0), 0.0);
    mat.insert(0, 1, 1.0);
    CHECK_FALSE(mat.empty());
    CHECK_EQ(mat.size(), 1);
    CHECK_EQ(mat.n(), 2);
    CHECK_EQ(mat(0, 1), 1.0);
    CHECK_EQ(mat(1, 0), 0.0);
    CHECK_EQ(mat(0, 0), 0.0);
    CHECK_EQ(mat(1, 1), 0.0);
    CHECK_THROWS_AS(mat(0, 2), std::out_of_range);
    mat.setDefaultValue(6.9);
    CHECK_EQ(mat(0, 0), 6.9);
  }
  Matrix testMat;
  testMat.insert(0, 1, 1.0);
  testMat.insert(1, 2, 2.0);
  testMat.insert(1, 3, 3.0);
  testMat.insert(2, 3, 4.0);
  testMat.insert(3, 4, 5.0);
  testMat.insert(0, 0, 6.0);
  SUBCASE("Copy and move") {
    auto copy = testMat;
    CHECK_EQ(testMat, copy);
    auto moved = std::move(copy);
    CHECK(copy.empty());
    CHECK_EQ(testMat, moved);
  }
  SUBCASE("Save and read") {
    auto path = "./data/sparse_matrix.dsm";
    testMat.save(path);
    Matrix mat(path);
    CHECK_EQ(testMat, mat);
  }
  SUBCASE("row getter") {
    std::map<Id, double> row;
    row = testMat.row(0);
    CHECK_EQ(row.size(), 2);
    CHECK_EQ(row[0], 6.0);
    CHECK_EQ(row[1], 1.0);
    row = testMat.row(1);
    CHECK_EQ(row.size(), 2);
    CHECK_EQ(row[2], 2.0);
    CHECK_EQ(row[3], 3.0);
    row = testMat.row(2);
    CHECK_EQ(row.size(), 1);
    CHECK_EQ(row[3], 4.0);
    row = testMat.row(3);
    CHECK_EQ(row.size(), 1);
    CHECK_EQ(row[4], 5.0);
    row = testMat.row(4);
    CHECK(row.empty());
    CHECK_THROWS_AS(testMat.row(5), std::out_of_range);
  }
  SUBCASE("normalize rows") {
    auto normalizedMat = testMat;
    normalizedMat.normalize();
    CHECK_EQ(normalizedMat(0, 0), testMat(0, 0) / 7.);
    CHECK_EQ(normalizedMat(0, 1), testMat(0, 1) / 7.);
    CHECK_EQ(normalizedMat(1, 2), testMat(1, 2) / 5.);
    CHECK_EQ(normalizedMat(1, 3), doctest::Approx(testMat(1, 3) / 5.));
    CHECK_EQ(normalizedMat(2, 3), testMat(2, 3) / 4.);
    CHECK_EQ(normalizedMat(3, 4), testMat(3, 4) / 5.);
  }
  SUBCASE("col getter") {
    std::map<Id, double> col;
    col = testMat.col(0);
    CHECK_EQ(col.size(), 1);
    CHECK_EQ(col[0], 6.0);
    col = testMat.col(1);
    CHECK_EQ(col.size(), 1);
    CHECK_EQ(col[0], 1.0);
    col = testMat.col(2);
    CHECK_EQ(col.size(), 1);
    CHECK_EQ(col[1], 2.0);
    col = testMat.col(3);
    CHECK_EQ(col.size(), 2);
    CHECK_EQ(col[1], 3.0);
    CHECK_EQ(col[2], 4.0);
    col = testMat.col(4);
    CHECK_EQ(col.size(), 1);
    CHECK_EQ(col[3], 5.0);
    CHECK_THROWS_AS(testMat.col(5), std::out_of_range);
  }
  SUBCASE("normalize cols") {
    auto normalizedMat = testMat;
    normalizedMat.normalize(false);
    CHECK_EQ(normalizedMat(0, 0), testMat(0, 0) / 6.);
    CHECK_EQ(normalizedMat(0, 1), testMat(0, 1) / 1.);
    CHECK_EQ(normalizedMat(1, 2), testMat(1, 2) / 2.);
    CHECK_EQ(normalizedMat(1, 3), doctest::Approx(testMat(1, 3) / 7.));
    CHECK_EQ(normalizedMat(2, 3), testMat(2, 3) / 7.);
    CHECK_EQ(normalizedMat(3, 4), testMat(3, 4) / 5.);
  }
}