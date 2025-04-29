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
    mat.insert(0, 1, 1.0);
    CHECK_FALSE(mat.empty());
    CHECK_EQ(mat.size(), 1);
    CHECK_EQ(mat.n(), 2);
    CHECK_EQ(mat(0, 1), 1.0);
    CHECK_EQ(mat(1, 0), 0.0);
    CHECK_EQ(mat(0, 0), 0.0);
    CHECK_EQ(mat(1, 1), 0.0);
    CHECK_THROWS_AS(mat(0, 2), std::out_of_range);
  }
}