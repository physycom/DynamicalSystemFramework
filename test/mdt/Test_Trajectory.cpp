#include "../../src/dsf/mdt/Trajectory.hpp"
#include "../../src/dsf/mdt/PointsCluster.hpp"
#include "../../src/dsf/geometry/Point.hpp"

#include "doctest.h"
#include <thread>
#include <chrono>

using namespace dsf::mdt;
using namespace dsf::geometry;

TEST_CASE("Trajectory basic functionality") {
    SUBCASE("Empty trajectory") {
        Trajectory<Point> traj;
        CHECK(traj.empty());
        CHECK_EQ(traj.size(), 0);
    }

    SUBCASE("Add points to trajectory") {
        Trajectory<Point> traj;
        traj.addPoint(1000, Point(10.0, 20.0));
        traj.addPoint(2000, Point(10.1, 20.1));
        
        CHECK_EQ(traj.size(), 2);
        CHECK_FALSE(traj.empty());
    }

    SUBCASE("Duplicate timestamp throws") {
        Trajectory<Point> traj;
        traj.addPoint(1000, Point(10.0, 20.0));
        CHECK_THROWS_AS(traj.addPoint(1000, Point(10.1, 20.1)), std::invalid_argument);
    }
}

TEST_CASE("Trajectory filter function") {
    SUBCASE("Empty trajectory filter") {
        Trajectory<Point> traj;
        auto filtered = traj.filter(50.0);
        CHECK(filtered.empty());
    }

    SUBCASE("Single point trajectory") {
        Trajectory<Point> traj;
        traj.addPoint(1000, Point(44.0, 11.0));
        
        auto filtered = traj.filter(50.0);
        CHECK_EQ(filtered.size(), 1);
    }

    SUBCASE("Points within cluster radius") {
        Trajectory<Point> traj;
        // Add points very close together (within ~10m)
        traj.addPoint(1000, Point(44.0, 11.0));
        traj.addPoint(1010, Point(44.0001, 11.0001)); // ~15m away
        traj.addPoint(1020, Point(44.0002, 11.0002)); // ~30m away from first
        
        // With cluster radius of 50m, all should be in one cluster
        auto filtered = traj.filter(50.0);
        CHECK_EQ(filtered.size(), 1);
        
        // The cluster should contain 3 points
        auto const& clusters = filtered.points();
        auto const& firstCluster = clusters.begin()->second;
        CHECK_EQ(firstCluster.size(), 3);
    }

    SUBCASE("Points exceeding cluster radius create new clusters") {
        Trajectory<Point> traj;
        // Add points far apart
        traj.addPoint(1000, Point(44.0, 11.0));
        traj.addPoint(2000, Point(44.001, 11.001)); // ~140m away
        traj.addPoint(3000, Point(44.002, 11.002)); // ~140m from previous
        
        // With cluster radius of 50m, should create multiple clusters
        auto filtered = traj.filter(50.0);
        CHECK_GT(filtered.size(), 1);
    }

    SUBCASE("Mixed clustering scenario") {
        Trajectory<Point> traj;
        std::time_t t = 1000;
        
        // First cluster: 3 close points
        traj.addPoint(t, Point(44.0, 11.0));
        t += 10;
        traj.addPoint(t, Point(44.0001, 11.0001));
        t += 10;
        traj.addPoint(t, Point(44.0002, 11.0001));
        
        // Move far away
        t += 100;
        traj.addPoint(t, Point(44.01, 11.01)); // ~1.4 km away
        
        // Second cluster: 2 close points
        t += 10;
        traj.addPoint(t, Point(44.0101, 11.0101));
        
        auto filtered = traj.filter(100.0); // 100m cluster radius
        
        CHECK_GE(filtered.size(), 2); // At least 2 clusters
    }
}

TEST_CASE("Haversine distance function") {
    SUBCASE("Same point") {
        Point p1(44.0, 11.0);
        Point p2(44.0, 11.0);
        double dist = haversine_m(p1, p2);
        CHECK_EQ(dist, doctest::Approx(0.0).epsilon(0.1));
    }

    SUBCASE("Known distance") {
        // Approximately 1 degree of latitude ≈ 111 km
        Point p1(44.0, 11.0);
        Point p2(45.0, 11.0); // 1 degree north
        double dist = haversine_m(p1, p2);
        CHECK_EQ(dist, doctest::Approx(111000.0).epsilon(1000.0)); // ~111 km
    }

    SUBCASE("Small distances") {
        Point p1(44.0, 11.0);
        Point p2(44.0001, 11.0001); // Very small displacement
        double dist = haversine_m(p1, p2);
        CHECK_LT(dist, 20.0); // Should be less than 20 meters
        CHECK_GT(dist, 10.0); // Should be more than 10 meters
    }
}

TEST_CASE("Trajectory of PointsClusters") {
    SUBCASE("Create trajectory of clusters") {
        Trajectory<PointsCluster> traj;
        
        PointsCluster cluster1;
        cluster1.addPoint(1000, Point(44.0, 11.0));
        cluster1.addPoint(1010, Point(44.0001, 11.0001));
        
        PointsCluster cluster2;
        cluster2.addPoint(2000, Point(45.0, 12.0));
        cluster2.addPoint(2010, Point(45.0001, 12.0001));
        
        traj.addPoint(1000, cluster1);
        traj.addPoint(2000, cluster2);
        
        CHECK_EQ(traj.size(), 2);
        CHECK_FALSE(traj.empty());
    }
}
