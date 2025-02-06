
#include "AdjacencyMatrix.hpp"
#include "Graph.hpp"

#include "doctest.h"

using namespace dsm;

TEST_CASE("Test default construction and insertion") {
  AdjacencyMatrix adj;
  adj.insert(0, 1);
  auto offsets = test::offsets(adj);
  auto indices = test::indices(adj);
  CHECK(offsets.size() == 2);
  CHECK(offsets[0] == 0);
  CHECK(offsets[1] == 1);
  CHECK(indices.size() == 1);
  CHECK(indices[0] == 1);
  CHECK(adj.nRows() == 1);
  CHECK(adj.nCols() == 2);

  adj.insert(1, 2);
  adj.insert(1, 3);
  adj.insert(2, 3);
  adj.insert(3, 4);
  offsets = test::offsets(adj);
  indices = test::indices(adj);
  CHECK(offsets.size() == 5);
  CHECK(offsets[0] == 0);
  CHECK(offsets[1] == 1);
  CHECK(offsets[2] == 3);
  CHECK(offsets[3] == 4);
  CHECK(offsets[4] == 5);
  CHECK(indices.size() == 5);
  CHECK(indices[0] == 1);
  CHECK(indices[1] == 2);
  CHECK(indices[2] == 3);
  CHECK(indices[3] == 3);
  CHECK(indices[4] == 4);
  CHECK(adj.nCols() == 5);
  CHECK(adj.nRows() == 4);

  adj.insert(0, 0);
  offsets = test::offsets(adj);
  indices = test::indices(adj);
  CHECK(offsets.size() == 5);
  CHECK(offsets[0] == 0);
  CHECK(offsets[1] == 2);
  CHECK(offsets[2] == 4);
  CHECK(offsets[3] == 5);
  CHECK(offsets[4] == 6);
  CHECK(indices.size() == 6);
  CHECK(indices[0] == 1);
  CHECK(indices[1] == 0);
  CHECK(indices[2] == 2);
  CHECK(indices[3] == 3);
  CHECK(indices[4] == 3);
  CHECK(indices[5] == 4);
  CHECK(adj.nCols() == 5);
  CHECK(adj.nRows() == 4);

  SUBCASE("Test contains") {
    CHECK(adj(0, 1));
    CHECK(adj(1, 2));
    CHECK(adj(1, 3));
    CHECK(adj(2, 3));
    CHECK(adj(3, 4));
    CHECK_FALSE(adj(0, 2));
    CHECK_FALSE(adj(2, 0));
    CHECK_FALSE(adj(3, 3));
    CHECK_THROWS(adj(5, 0));
    CHECK_THROWS(adj(10, 0));
    CHECK_THROWS(adj(0, 5));
    CHECK_THROWS(adj(0, 10));
  }
  SUBCASE("Test getCol") {
    auto col0 = adj.getCol(0);
    CHECK(col0.size() == 1);
    CHECK(col0[0] == 0);
    auto col1 = adj.getCol(1);
    CHECK(col1.size() == 1);
    CHECK(col1[0] == 0);
    auto col2 = adj.getCol(2);
    CHECK(col2.size() == 1);
    CHECK(col2[0] == 1);
    auto col3 = adj.getCol(3);
    CHECK(col3.size() == 2);
    CHECK(col3[0] == 1);
    CHECK(col3[1] == 2);
  }
  SUBCASE("Test getRow") {
    auto row0 = adj.getRow(0);
    CHECK(row0.size() == 2);
    CHECK(row0[0] == 1);
    CHECK(row0[1] == 0);
    auto row1 = adj.getRow(1);
    CHECK(row1.size() == 2);
    CHECK(row1[0] == 2);
    CHECK(row1[1] == 3);
    auto row2 = adj.getRow(2);
    CHECK(row2.size() == 1);
    CHECK(row2[0] == 3);
    auto row3 = adj.getRow(3);
    CHECK(row3.size() == 1);
    CHECK(row3[0] == 4);
  }
  SUBCASE("Test getInDegreeVector") {
    auto inDegreeVector = adj.getInDegreeVector();
    CHECK(inDegreeVector.size() == 5);
    CHECK(inDegreeVector[0] == 1);
    CHECK(inDegreeVector[1] == 1);
    CHECK(inDegreeVector[2] == 1);
    CHECK(inDegreeVector[3] == 2);
    CHECK(inDegreeVector[4] == 1);
  }
  SUBCASE("Test getOutDegreeVector") {
    auto outDegreeVector = adj.getOutDegreeVector();
    CHECK(outDegreeVector.size() == 4);
    CHECK(outDegreeVector[0] == 2);
    CHECK(outDegreeVector[1] == 2);
    CHECK(outDegreeVector[2] == 1);
    CHECK(outDegreeVector[3] == 1);
  }
}

TEST_CASE("Test construction from edge map") {
  Graph g;
  g.addEdge<Street>(0, std::make_pair<Id, Id>(0, 1));
  g.addEdge<Street>(1, std::make_pair<Id, Id>(1, 2));
  g.addEdge<Street>(2, std::make_pair<Id, Id>(1, 3));
  g.addEdge<Street>(3, std::make_pair<Id, Id>(2, 3));
  g.addEdge<Street>(4, std::make_pair<Id, Id>(3, 4));
  AdjacencyMatrix adj(g.streetSet());

  auto offsets = test::offsets(adj);
  auto indices = test::indices(adj);
  CHECK(offsets.size() == 5);
  CHECK(offsets[0] == 0);
  CHECK(offsets[1] == 1);
  CHECK(offsets[2] == 3);
  CHECK(offsets[3] == 4);
  CHECK(offsets[4] == 5);
  CHECK(indices.size() == 5);
  CHECK(indices[0] == 1);
  CHECK(indices[1] == 3);
  CHECK(indices[2] == 2);
  CHECK(indices[3] == 3);
  CHECK(indices[4] == 4);
  CHECK(adj.nCols() == 5);
  CHECK(adj.nRows() == 4);

  SUBCASE("Test contains") {
    CHECK(adj(0, 1));
    CHECK(adj(1, 2));
    CHECK(adj(1, 3));
    CHECK(adj(2, 3));
    CHECK(adj(3, 4));
    CHECK_FALSE(adj(0, 2));
    CHECK_FALSE(adj(2, 0));
    CHECK_FALSE(adj(3, 3));
    CHECK_THROWS(adj(5, 0));
    CHECK_THROWS(adj(10, 0));
    CHECK_THROWS(adj(0, 5));
    CHECK_THROWS(adj(0, 10));
  }
  SUBCASE("Test getCol") {
    auto col0 = adj.getCol(0);
    CHECK(col0.size() == 0);
    auto col1 = adj.getCol(1);
    CHECK(col1.size() == 1);
    CHECK(col1[0] == 0);
    auto col2 = adj.getCol(2);
    CHECK(col2.size() == 1);
    CHECK(col2[0] == 1);
    auto col3 = adj.getCol(3);
    CHECK(col3.size() == 2);
    CHECK(col3[0] == 1);
    CHECK(col3[1] == 2);
  }
  SUBCASE("Test getRow") {
    auto row0 = adj.getRow(0);
    CHECK(row0.size() == 1);
    CHECK(row0[0] == 1);
    auto row1 = adj.getRow(1);
    CHECK(row1.size() == 2);
    CHECK(row1[0] == 3);
    CHECK(row1[1] == 2);
    auto row2 = adj.getRow(2);
    CHECK(row2.size() == 1);
    CHECK(row2[0] == 3);
    auto row3 = adj.getRow(3);
    CHECK(row3.size() == 1);
    CHECK(row3[0] == 4);
  }
  SUBCASE("Test getInDegreeVector") {
    auto inDegreeVector = adj.getInDegreeVector();
    CHECK(inDegreeVector.size() == 5);
    CHECK(inDegreeVector[0] == 0);
    CHECK(inDegreeVector[1] == 1);
    CHECK(inDegreeVector[2] == 1);
    CHECK(inDegreeVector[3] == 2);
    CHECK(inDegreeVector[4] == 1);
  }
  SUBCASE("Test getOutDegreeVector") {
    auto outDegreeVector = adj.getOutDegreeVector();
    CHECK(outDegreeVector.size() == 4);
    CHECK(outDegreeVector[0] == 1);
    CHECK(outDegreeVector[1] == 2);
    CHECK(outDegreeVector[2] == 1);
    CHECK(outDegreeVector[3] == 1);
  }
}
