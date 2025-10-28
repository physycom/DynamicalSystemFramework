#include "../../src/dsf/mdt/PointsCluster.hpp"
#include "../../src/dsf/geometry/Point.hpp"

#include "doctest.h"

using namespace dsf::mdt;
using namespace dsf::geometry;

TEST_CASE("PointsCluster basic functionality") {
    SUBCASE("Empty cluster throws on centroid") {
        PointsCluster cluster;
        CHECK_THROWS_AS(cluster.centroid(), std::runtime_error);
    }

    SUBCASE("Single point cluster") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(5.0, 10.0));
        
        Point centroid = cluster.centroid();
        CHECK_EQ(centroid.x(), 5.0);
        CHECK_EQ(centroid.y(), 10.0);
        CHECK_EQ(cluster.size(), 1);
    }

    SUBCASE("Duplicate timestamp throws") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.0, 2.0));
        CHECK_THROWS_AS(cluster.addPoint(1000, Point(3.0, 4.0)), std::invalid_argument);
    }
}

TEST_CASE("PointsCluster median centroid - odd number of points") {
    SUBCASE("Three points in a line") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.0, 1.0));
        cluster.addPoint(2000, Point(3.0, 3.0));
        cluster.addPoint(3000, Point(5.0, 5.0));
        
        Point centroid = cluster.centroid();
        // Median of [1.0, 3.0, 5.0] is 3.0
        CHECK_EQ(centroid.x(), 3.0);
        CHECK_EQ(centroid.y(), 3.0);
        // Timestamps
        CHECK_EQ(cluster.firstTimestamp(), 1000);
        CHECK_EQ(cluster.lastTimestamp(), 3000);
    }

    SUBCASE("Five points with mixed coordinates") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.0, 10.0));
        cluster.addPoint(2000, Point(2.0, 20.0));
        cluster.addPoint(3000, Point(3.0, 30.0));
        cluster.addPoint(4000, Point(4.0, 40.0));
        cluster.addPoint(5000, Point(5.0, 50.0));
        
        Point centroid = cluster.centroid();
        // Median of [1.0, 2.0, 3.0, 4.0, 5.0] is 3.0
        // Median of [10.0, 20.0, 30.0, 40.0, 50.0] is 30.0
        CHECK_EQ(centroid.x(), 3.0);
        CHECK_EQ(centroid.y(), 30.0);
    }

    SUBCASE("Three points - unsorted order") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(10.0, 5.0));
        cluster.addPoint(2000, Point(2.0, 15.0));
        cluster.addPoint(3000, Point(6.0, 10.0));
        
        Point centroid = cluster.centroid();
        // Median of [10.0, 2.0, 6.0] → sorted [2.0, 6.0, 10.0] → median is 6.0
        // Median of [5.0, 15.0, 10.0] → sorted [5.0, 10.0, 15.0] → median is 10.0
        CHECK_EQ(centroid.x(), 6.0);
        CHECK_EQ(centroid.y(), 10.0);
    }
}

TEST_CASE("PointsCluster median centroid - even number of points") {
    SUBCASE("Two points") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(2.0, 4.0));
        cluster.addPoint(2000, Point(8.0, 12.0));
        
        Point centroid = cluster.centroid();
        // Median of [2.0, 8.0] is (2.0 + 8.0) / 2 = 5.0
        // Median of [4.0, 12.0] is (4.0 + 12.0) / 2 = 8.0
        CHECK_EQ(centroid.x(), 5.0);
        CHECK_EQ(centroid.y(), 8.0);
    }

    SUBCASE("Four points") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.0, 10.0));
        cluster.addPoint(2000, Point(2.0, 20.0));
        cluster.addPoint(3000, Point(3.0, 30.0));
        cluster.addPoint(4000, Point(4.0, 40.0));
        
        Point centroid = cluster.centroid();
        // Median of [1.0, 2.0, 3.0, 4.0] is (2.0 + 3.0) / 2 = 2.5
        // Median of [10.0, 20.0, 30.0, 40.0] is (20.0 + 30.0) / 2 = 25.0
        CHECK_EQ(centroid.x(), 2.5);
        CHECK_EQ(centroid.y(), 25.0);
    }

    SUBCASE("Six points - unsorted") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(9.0, 1.0));
        cluster.addPoint(2000, Point(3.0, 6.0));
        cluster.addPoint(3000, Point(7.0, 4.0));
        cluster.addPoint(4000, Point(1.0, 8.0));
        cluster.addPoint(5000, Point(5.0, 2.0));
        cluster.addPoint(6000, Point(11.0, 10.0));
        
        Point centroid = cluster.centroid();
        // X: [9.0, 3.0, 7.0, 1.0, 5.0, 11.0] → sorted [1.0, 3.0, 5.0, 7.0, 9.0, 11.0]
        // Median is (5.0 + 7.0) / 2 = 6.0
        // Y: [1.0, 6.0, 4.0, 8.0, 2.0, 10.0] → sorted [1.0, 2.0, 4.0, 6.0, 8.0, 10.0]
        // Median is (4.0 + 6.0) / 2 = 5.0
        CHECK_EQ(centroid.x(), 6.0);
        CHECK_EQ(centroid.y(), 5.0);
    }
}

TEST_CASE("PointsCluster centroid caching") {
    SUBCASE("Centroid is cached after first call") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.0, 2.0));
        cluster.addPoint(2000, Point(3.0, 4.0));
        cluster.addPoint(3000, Point(5.0, 6.0));
        
        // First call computes the centroid
        Point centroid1 = cluster.centroid();
        // Second call should return cached value
        Point centroid2 = cluster.centroid();
        
        CHECK_EQ(centroid1.x(), centroid2.x());
        CHECK_EQ(centroid1.y(), centroid2.y());
        CHECK_EQ(centroid1.x(), 3.0);
        CHECK_EQ(centroid1.y(), 4.0);
    }
}

TEST_CASE("PointsCluster with negative and mixed coordinates") {
    SUBCASE("All negative coordinates") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(-5.0, -10.0));
        cluster.addPoint(2000, Point(-3.0, -8.0));
        cluster.addPoint(3000, Point(-1.0, -6.0));
        
        Point centroid = cluster.centroid();
        // Median of [-5.0, -3.0, -1.0] is -3.0
        // Median of [-10.0, -8.0, -6.0] is -8.0
        CHECK_EQ(centroid.x(), -3.0);
        CHECK_EQ(centroid.y(), -8.0);
    }

    SUBCASE("Mixed positive and negative") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(-2.0, 5.0));
        cluster.addPoint(2000, Point(0.0, -3.0));
        cluster.addPoint(3000, Point(2.0, 1.0));
        
        Point centroid = cluster.centroid();
        // Median of [-2.0, 0.0, 2.0] is 0.0
        // Median of [5.0, -3.0, 1.0] → sorted [-3.0, 1.0, 5.0] → median is 1.0
        CHECK_EQ(centroid.x(), 0.0);
        CHECK_EQ(centroid.y(), 1.0);
    }
}

TEST_CASE("PointsCluster with floating point precision") {
    SUBCASE("Very close values") {
        PointsCluster cluster;
        cluster.addPoint(1000, Point(1.00001, 2.00001));
        cluster.addPoint(2000, Point(1.00002, 2.00002));
        cluster.addPoint(3000, Point(1.00003, 2.00003));
        
        Point centroid = cluster.centroid();
        CHECK_EQ(centroid.x(), doctest::Approx(1.00002).epsilon(0.00001));
        CHECK_EQ(centroid.y(), doctest::Approx(2.00002).epsilon(0.00001));
    }
}
