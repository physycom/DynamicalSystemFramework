#include "../src/dsf/base/SparseMatrix.hpp"

#include "doctest.h"

using namespace dsf;

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
    auto path = "./data/sparse_matrix.dsf";
    testMat.save(path);
    Matrix mat(path);
    CHECK_EQ(testMat, mat);
  }
  SUBCASE("csv format") {
    Matrix readMat("./data/sparsematrix.csv", "csv");
    CHECK_EQ(readMat(0, 1), 3.270491);
    CHECK_EQ(readMat(0, 3), 10.995416);
    CHECK_EQ(readMat(0, 4), 53.969164);
    CHECK_EQ(readMat(1, 1), 44.926008);
    CHECK_EQ(readMat(1, 3), 6.705062);
    CHECK_EQ(readMat(2, 1), 76.972337);
    CHECK_EQ(readMat(3, 3), 34.396410);
    CHECK_EQ(readMat(4, 0), 7.659321);
    CHECK_EQ(readMat(4, 2), 44.597329);
    CHECK_EQ(readMat(4, 3), 7.036792);
    CHECK_EQ(readMat(4, 4), 6.309386);
    CHECK_EQ(readMat.size(), 11);
    CHECK_EQ(readMat.n(), 5);
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
    double value{5.0};
    auto normalizedMat = testMat;
    normalizedMat.normalize(1, value);
    CHECK_EQ(normalizedMat(0, 0), value * testMat(0, 0) / 7.);
    CHECK_EQ(normalizedMat(0, 1), value * testMat(0, 1) / 7.);
    CHECK_EQ(normalizedMat(1, 2), value * testMat(1, 2) / 5.);
    CHECK_EQ(normalizedMat(1, 3), doctest::Approx(value * testMat(1, 3) / 5.));
    CHECK_EQ(normalizedMat(2, 3), value * testMat(2, 3) / 4.);
    CHECK_EQ(normalizedMat(3, 4), value * testMat(3, 4) / 5.);
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
    double value{6.0};
    auto normalizedMat = testMat;
    normalizedMat.normalize(false, value);
    CHECK_EQ(normalizedMat(0, 0), value * testMat(0, 0) / 6.);
    CHECK_EQ(normalizedMat(0, 1), value * testMat(0, 1) / 1.);
    CHECK_EQ(normalizedMat(1, 2), value * testMat(1, 2) / 2.);
    CHECK_EQ(normalizedMat(1, 3), doctest::Approx(value * testMat(1, 3) / 7.));
    CHECK_EQ(normalizedMat(2, 3), value * testMat(2, 3) / 7.);
    CHECK_EQ(normalizedMat(3, 4), value * testMat(3, 4) / 5.);
  }
}