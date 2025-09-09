// #include "AdjacencyMatrix.hpp"
// #include "RoadNetwork.hpp"

// #include "doctest.h"

// using namespace dsf;

// TEST_CASE("Test default construction and insertion") {
//   AdjacencyMatrix adj;
//   adj.insert(0, 1);
//   auto offsets = test::offsets(adj);
//   auto indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 3);
//   CHECK_EQ(offsets[0], 0);
//   CHECK_EQ(offsets[1], 1);
//   CHECK_EQ(indices.size(), 1);
//   CHECK_EQ(indices[0], 1);
//   CHECK_EQ(adj.n(), 2);

//   adj.insert(1, 2);
//   adj.insert(1, 3);
//   adj.insert(2, 3);
//   adj.insert(3, 4);
//   offsets = test::offsets(adj);
//   indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 6);
//   CHECK_EQ(offsets[0], 0);
//   CHECK_EQ(offsets[1], 1);
//   CHECK_EQ(offsets[2], 3);
//   CHECK_EQ(offsets[3], 4);
//   CHECK_EQ(offsets[4], 5);
//   CHECK_EQ(indices.size(), 5);
//   CHECK_EQ(indices[0], 1);
//   CHECK_EQ(indices[1], 2);
//   CHECK_EQ(indices[2], 3);
//   CHECK_EQ(indices[3], 3);
//   CHECK_EQ(indices[4], 4);
//   CHECK_EQ(adj.n(), 5);

//   adj.insert(0, 0);
//   offsets = test::offsets(adj);
//   indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 6);
//   CHECK_EQ(offsets[0], 0);
//   CHECK_EQ(offsets[1], 2);
//   CHECK_EQ(offsets[2], 4);
//   CHECK_EQ(offsets[3], 5);
//   CHECK_EQ(offsets[4], 6);
//   CHECK_EQ(indices.size(), 6);
//   CHECK_EQ(indices[0], 1);
//   CHECK_EQ(indices[1], 0);
//   CHECK_EQ(indices[2], 2);
//   CHECK_EQ(indices[3], 3);
//   CHECK_EQ(indices[4], 3);
//   CHECK_EQ(indices[5], 4);
//   CHECK_EQ(adj.n(), 5);

//   SUBCASE("Test contains") {
//     CHECK(adj(0, 1));
//     CHECK(adj(1, 2));
//     CHECK(adj(1, 3));
//     CHECK(adj(2, 3));
//     CHECK(adj(3, 4));
//     CHECK_FALSE(adj(0, 2));
//     CHECK_FALSE(adj(2, 0));
//     CHECK_FALSE(adj(3, 3));
//     CHECK_THROWS(adj(5, 0));
//     CHECK_THROWS(adj(10, 0));
//     CHECK_THROWS(adj(0, 5));
//     CHECK_THROWS(adj(0, 10));
//   }
//   SUBCASE("Test getCol") {
//     auto col0 = adj.getCol(0);
//     CHECK_EQ(col0.size(), 1);
//     CHECK_EQ(col0[0], 0);
//     auto col1 = adj.getCol(1);
//     CHECK_EQ(col1.size(), 1);
//     CHECK_EQ(col1[0], 0);
//     auto col2 = adj.getCol(2);
//     CHECK_EQ(col2.size(), 1);
//     CHECK_EQ(col2[0], 1);
//     auto col3 = adj.getCol(3);
//     CHECK_EQ(col3.size(), 2);
//     CHECK_EQ(col3[0], 1);
//     CHECK_EQ(col3[1], 2);
//   }
//   SUBCASE("Test getRow") {
//     auto row0 = adj.getRow(0);
//     CHECK_EQ(row0.size(), 2);
//     CHECK_EQ(row0[0], 1);
//     CHECK_EQ(row0[1], 0);
//     auto row1 = adj.getRow(1);
//     CHECK_EQ(row1.size(), 2);
//     CHECK_EQ(row1[0], 2);
//     CHECK_EQ(row1[1], 3);
//     auto row2 = adj.getRow(2);
//     CHECK_EQ(row2.size(), 1);
//     CHECK_EQ(row2[0], 3);
//     auto row3 = adj.getRow(3);
//     CHECK_EQ(row3.size(), 1);
//     CHECK_EQ(row3[0], 4);
//   }
//   SUBCASE("Test getInDegreeVector") {
//     auto inDegreeVector = adj.getInDegreeVector();
//     CHECK_EQ(inDegreeVector.size(), adj.n());
//     CHECK_EQ(inDegreeVector[0], 1);
//     CHECK_EQ(inDegreeVector[1], 1);
//     CHECK_EQ(inDegreeVector[2], 1);
//     CHECK_EQ(inDegreeVector[3], 2);
//     CHECK_EQ(inDegreeVector[4], 1);
//   }
//   SUBCASE("Test getOutDegreeVector") {
//     auto outDegreeVector = adj.getOutDegreeVector();
//     CHECK_EQ(outDegreeVector.size(), 5);
//     CHECK_EQ(outDegreeVector[0], 2);
//     CHECK_EQ(outDegreeVector[1], 2);
//     CHECK_EQ(outDegreeVector[2], 1);
//     CHECK_EQ(outDegreeVector[3], 1);
//     CHECK_EQ(outDegreeVector[4], 0);
//   }
// }
// TEST_CASE("Test construction from edge map") {
//   RoadNetwork g;
//   g.addEdge<Street>(0, std::make_pair<Id, Id>(0, 1));
//   g.addEdge<Street>(1, std::make_pair<Id, Id>(1, 2));
//   g.addEdge<Street>(2, std::make_pair<Id, Id>(1, 3));
//   g.addEdge<Street>(3, std::make_pair<Id, Id>(2, 3));
//   g.addEdge<Street>(4, std::make_pair<Id, Id>(3, 4));
//   AdjacencyMatrix adj(g.edges());

//   auto offsets = test::offsets(adj);
//   auto indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 6);
//   CHECK_EQ(offsets[0], 0);
//   CHECK_EQ(offsets[1], 1);
//   CHECK_EQ(offsets[2], 3);
//   CHECK_EQ(offsets[3], 4);
//   CHECK_EQ(offsets[4], 5);
//   CHECK_EQ(indices.size(), 5);
//   CHECK_EQ(indices[0], 1);
//   CHECK_EQ(indices[1], 3);
//   CHECK_EQ(indices[2], 2);
//   CHECK_EQ(indices[3], 3);
//   CHECK_EQ(indices[4], 4);
//   CHECK_EQ(adj.n(), 5);

//   SUBCASE("Test contains") {
//     CHECK(adj(0, 1));
//     CHECK(adj(1, 2));
//     CHECK(adj(1, 3));
//     CHECK(adj(2, 3));
//     CHECK(adj(3, 4));
//     CHECK_FALSE(adj(0, 2));
//     CHECK_FALSE(adj(2, 0));
//     CHECK_FALSE(adj(3, 3));
//     CHECK_THROWS(adj(5, 0));
//     CHECK_THROWS(adj(10, 0));
//     CHECK_THROWS(adj(0, 5));
//     CHECK_THROWS(adj(0, 10));
//   }
//   SUBCASE("Test getCol") {
//     auto col0 = adj.getCol(0);
//     CHECK_EQ(col0.size(), 0);
//     auto col1 = adj.getCol(1);
//     CHECK_EQ(col1.size(), 1);
//     CHECK_EQ(col1[0], 0);
//     auto col2 = adj.getCol(2);
//     CHECK_EQ(col2.size(), 1);
//     CHECK_EQ(col2[0], 1);
//     auto col3 = adj.getCol(3);
//     CHECK_EQ(col3.size(), 2);
//     CHECK_EQ(col3[0], 2);
//     CHECK_EQ(col3[1], 1);
//   }
//   SUBCASE("Test getRow") {
//     auto row0 = adj.getRow(0);
//     CHECK_EQ(row0.size(), 1);
//     CHECK_EQ(row0[0], 1);
//     auto row1 = adj.getRow(1);
//     CHECK_EQ(row1.size(), 2);
//     CHECK_EQ(row1[0], 3);
//     CHECK_EQ(row1[1], 2);
//     auto row2 = adj.getRow(2);
//     CHECK_EQ(row2.size(), 1);
//     CHECK_EQ(row2[0], 3);
//     auto row3 = adj.getRow(3);
//     CHECK_EQ(row3.size(), 1);
//     CHECK_EQ(row3[0], 4);
//   }
//   SUBCASE("Test getInDegreeVector") {
//     auto inDegreeVector = adj.getInDegreeVector();
//     CHECK_EQ(inDegreeVector.size(), 5);
//     CHECK_EQ(inDegreeVector[0], 0);
//     CHECK_EQ(inDegreeVector[1], 1);
//     CHECK_EQ(inDegreeVector[2], 1);
//     CHECK_EQ(inDegreeVector[3], 2);
//     CHECK_EQ(inDegreeVector[4], 1);
//   }
//   SUBCASE("Test getOutDegreeVector") {
//     auto outDegreeVector = adj.getOutDegreeVector();
//     CHECK_EQ(outDegreeVector.size(), 5);
//     CHECK_EQ(outDegreeVector[0], 1);
//     CHECK_EQ(outDegreeVector[1], 2);
//     CHECK_EQ(outDegreeVector[2], 1);
//     CHECK_EQ(outDegreeVector[3], 1);
//     CHECK_EQ(outDegreeVector[4], 0);
//   }
//   SUBCASE("operator ==") {
//     AdjacencyMatrix adj2;
//     CHECK_NE(adj, adj2);
//   }
//   SUBCASE("Test save and read") {
//     auto const filePath = "./data/test.adj";
//     adj.save(filePath);
//     AdjacencyMatrix adj2(filePath);
//     CHECK_EQ(adj, adj2);
//   }
//   SUBCASE("Test clearRow") {
//     adj.clearRow(1);
//     CHECK_FALSE(adj(1, 2));
//     CHECK_FALSE(adj(1, 3));
//   }
//   SUBCASE("Test clearCol") {
//     adj.clearCol(3);
//     CHECK_FALSE(adj(1, 3));
//     CHECK_FALSE(adj(2, 3));
//   }
//   SUBCASE("Test clear") {
//     adj.clear();
//     CHECK_EQ(adj, AdjacencyMatrix{});
//   }
// }

// TEST_CASE("Test insertion of random values") {
//   AdjacencyMatrix adj;
//   adj.insert(4, 2);
//   auto offsets = test::offsets(adj);
//   auto indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 6);
//   std::for_each(
//       offsets.begin(), offsets.begin() + 5, [](auto value) { CHECK(value == 0); });
//   CHECK_EQ(offsets[5], 1);
//   CHECK_EQ(indices.size(), 1);
//   CHECK_EQ(indices[0], 2);

//   adj.insert(63, 268);
//   offsets = test::offsets(adj);
//   indices = test::indices(adj);
//   CHECK(offsets.size() == 270);
//   std::for_each(
//       offsets.begin() + 5, offsets.begin() + 63, [](auto value) { CHECK(value == 1); });
//   CHECK_EQ(offsets[64], 2);
//   CHECK_EQ(indices.size(), 2);
//   CHECK_EQ(indices[1], 268);

//   adj.insert(2, 3);
//   offsets = test::offsets(adj);
//   indices = test::indices(adj);
//   CHECK_EQ(offsets.size(), 270);
//   CHECK_EQ(offsets[0], 0);
//   CHECK_EQ(offsets[2], 0);
//   CHECK_EQ(offsets[3], 1);
//   CHECK_EQ(offsets[4], 1);
//   CHECK_EQ(offsets[5], 2);
//   CHECK_EQ(offsets[63], 2);
//   CHECK_EQ(offsets[64], 3);
//   CHECK_EQ(indices.size(), 3);
//   CHECK_EQ(indices[0], 3);
//   CHECK_EQ(indices[1], 2);
//   CHECK_EQ(indices[2], 268);
// }
