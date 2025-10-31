#include "dsf/mdt/TrajectoryCollection.hpp"

#include <filesystem>
#include <fstream>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

using namespace dsf::mdt;
using namespace dsf::geometry;

// Helper function to create a test CSV file
void createTestCSV(std::string const& filename, std::string const& content) {
  std::ofstream file(filename);
  file << content;
  file.close();
}

// Helper function to read CSV file contents
std::string readFile(std::string const& filename) {
  std::ifstream file(filename);
  std::string content((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  return content;
}

TEST_CASE("TrajectoryCollection - Default constructor") {
  TrajectoryCollection collection;
  // Should not crash, collection is created but empty
  CHECK(true);
}

TEST_CASE("TrajectoryCollection - Constructor with non-existent file") {
  // Should throw when trying to load non-existent file
  CHECK_THROWS(TrajectoryCollection("non_existent_file.csv"));
}

TEST_CASE("TrajectoryCollection - Import from CSV") {
  std::string testFile = "test_trajectory_import.csv";

  // Create a simple test CSV
  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n"
      "2;1500;45.4064;11.8767\n"
      "2;2500;45.4065;11.8768\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection;
  collection.import(testFile);

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);  // Successfully imported
}

TEST_CASE("TrajectoryCollection - Import with custom separator") {
  std::string testFile = "test_trajectory_comma.csv";

  // Create a test CSV with comma separator
  std::string csvContent =
      "uid,timestamp,lat,lon\n"
      "1,1000,44.4949,11.3426\n"
      "1,2000,44.4950,11.3427\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection;
  collection.import(testFile, std::unordered_map<std::string, std::string>{}, ',');

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);  // Successfully imported with comma separator
}

TEST_CASE("TrajectoryCollection - Constructor with CSV file") {
  std::string testFile = "test_trajectory_constructor.csv";

  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection(testFile);

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);  // Successfully created with file
}

TEST_CASE("TrajectoryCollection - Export to CSV") {
  std::string importFile = "test_trajectory_import_export.csv";
  std::string exportFile = "test_trajectory_export.csv";

  // Create test data
  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n"
      "1;3000;44.4951;11.3428\n"
      "2;1500;45.4064;11.8767\n"
      "2;2500;45.4065;11.8768\n";
  createTestCSV(importFile, csvContent);

  TrajectoryCollection collection(importFile);
  collection.to_csv(exportFile);

  // Check that export file exists
  CHECK(std::filesystem::exists(exportFile));

  // Read and verify export file has content
  std::string exportContent = readFile(exportFile);
  CHECK_FALSE(exportContent.empty());
  CHECK(exportContent.find("uid") != std::string::npos);
  CHECK(exportContent.find("lon") != std::string::npos);
  CHECK(exportContent.find("lat") != std::string::npos);
  CHECK(exportContent.find("timestamp_in") != std::string::npos);
  CHECK(exportContent.find("timestamp_out") != std::string::npos);

  // Clean up
  std::filesystem::remove(importFile);
  std::filesystem::remove(exportFile);
}

TEST_CASE("TrajectoryCollection - Export with custom separator") {
  std::string importFile = "test_trajectory_import_custom.csv";
  std::string exportFile = "test_trajectory_export_custom.csv";

  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n";
  createTestCSV(importFile, csvContent);

  TrajectoryCollection collection(importFile);
  collection.to_csv(exportFile, ',');

  // Verify comma separator in export
  std::string exportContent = readFile(exportFile);
  CHECK(exportContent.find(',') != std::string::npos);

  // Clean up
  std::filesystem::remove(importFile);
  std::filesystem::remove(exportFile);
}

TEST_CASE("TrajectoryCollection - Export to invalid path") {
  TrajectoryCollection collection;

  // Try to export to an invalid path
  CHECK_THROWS_AS(collection.to_csv("/invalid/path/file.csv"), std::runtime_error);
}

TEST_CASE("TrajectoryCollection - Filter trajectories") {
  std::string testFile = "test_trajectory_filter.csv";

  // Create test data with points that should cluster
  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n"
      "1;3000;44.4951;11.3428\n"
      "2;1000;45.4064;11.8767\n"
      "2;2000;45.4065;11.8768\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection(testFile);

  // Apply filter (should not crash)
  collection.filter(0.5, 150.0);  // 500m radius, 150 km/h max speed

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);  // Filter completed successfully
}

TEST_CASE("TrajectoryCollection - Filter with different parameters") {
  std::string testFile = "test_trajectory_filter_params.csv";

  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;1000;44.4949;11.3426\n"
      "1;2000;44.4950;11.3427\n"
      "1;3000;44.4951;11.3428\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection(testFile);

  // Try different filter parameters
  collection.filter(1.0, 100.0);  // 1 km radius, 100 km/h

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);
}

TEST_CASE("TrajectoryCollection - Multiple users workflow") {
  std::string importFile = "test_trajectory_multiuser.csv";
  std::string exportFile = "test_trajectory_multiuser_export.csv";

  // Create data for multiple users with different trajectories
  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      "1;28800;44.4949;11.3426\n"  // User 1: at home
      "1;29700;44.4950;11.3427\n"
      "1;30600;44.4949;11.3426\n"
      "1;36000;44.5500;11.4000\n"  // User 1: at work
      "1;43200;44.5501;11.4001\n"
      "2;28800;45.4064;11.8767\n"  // User 2: different location
      "2;29700;45.4065;11.8768\n"
      "2;30600;45.4064;11.8767\n"
      "3;30000;44.4000;11.5000\n"  // User 3: another location
      "3;31000;44.4001;11.5001\n";
  createTestCSV(importFile, csvContent);

  // Import
  TrajectoryCollection collection(importFile);

  // Filter to identify stop points
  collection.filter(0.5, 150.0, 0);

  // Export filtered results
  collection.to_csv(exportFile);

  // Verify export exists and has data
  CHECK(std::filesystem::exists(exportFile));
  std::string exportContent = readFile(exportFile);
  CHECK_FALSE(exportContent.empty());

  // Should have entries for multiple users
  CHECK(exportContent.find("1") != std::string::npos);
  CHECK(exportContent.find("2") != std::string::npos);
  CHECK(exportContent.find("3") != std::string::npos);

  // Clean up
  std::filesystem::remove(importFile);
  std::filesystem::remove(exportFile);
}

TEST_CASE("TrajectoryCollection - Empty import") {
  std::string testFile = "test_trajectory_empty.csv";

  // Create CSV with only header
  std::string csvContent = "uid;timestamp;lat;lon\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection;
  collection.import(testFile);

  // Should not crash with empty data
  collection.filter(1.0, 150.0);

  // Clean up
  std::filesystem::remove(testFile);

  CHECK(true);
}

TEST_CASE("TrajectoryCollection - Large dataset") {
  std::string testFile = "test_trajectory_large.csv";

  // Create a larger dataset
  std::ofstream file(testFile);
  file << "uid;timestamp;lat;lon\n";

  // Generate 100 points for user 1
  for (int i = 0; i < 100; ++i) {
    file << "1;" << (1000 + i * 100) << ";" << (44.4949 + i * 0.0001) << ";"
         << (11.3426 + i * 0.0001) << "\n";
  }

  // Generate 100 points for user 2
  for (int i = 0; i < 100; ++i) {
    file << "2;" << (1000 + i * 100) << ";" << (45.4064 + i * 0.0001) << ";"
         << (11.8767 + i * 0.0001) << "\n";
  }
  file.close();

  TrajectoryCollection collection(testFile);

  // Filter should work on larger dataset
  collection.filter(1.0, 150.0);

  // Export should work
  std::string exportFile = "test_trajectory_large_export.csv";
  collection.to_csv(exportFile);

  CHECK(std::filesystem::exists(exportFile));

  // Clean up
  std::filesystem::remove(testFile);
  std::filesystem::remove(exportFile);
}

TEST_CASE("TrajectoryCollection - Comprehensive workflow") {
  std::string importFile = "test_comprehensive.csv";
  std::string exportFile = "test_comprehensive_export.csv";

  // Realistic trajectory data
  std::string csvContent =
      "uid;timestamp;lat;lon\n"
      // User 1: Home -> Work -> Home pattern
      "1;28800;44.4949;11.3426\n"  // 8:00 AM - Home
      "1;29100;44.4950;11.3427\n"
      "1;29400;44.4949;11.3426\n"
      "1;32400;44.5200;11.4000\n"  // 9:00 AM - Transit
      "1;36000;44.5500;11.4500\n"  // 10:00 AM - Work
      "1;39600;44.5501;11.4501\n"
      "1;43200;44.5500;11.4500\n"
      "1;64800;44.4949;11.3426\n"  // 6:00 PM - Home
      "1;68400;44.4950;11.3427\n"
      // User 2: Stay at one location
      "2;28800;45.4064;11.8767\n"
      "2;32400;45.4065;11.8768\n"
      "2;36000;45.4064;11.8767\n"
      "2;43200;45.4065;11.8768\n";
  createTestCSV(importFile, csvContent);

  // Step 1: Import
  TrajectoryCollection collection(importFile);

  // Step 2: Filter to identify stop points
  // Use 500m radius and 150 km/h max speed
  collection.filter(0.5, 150.0, 0);

  // Step 3: Export results
  collection.to_csv(exportFile);

  // Verify export
  CHECK(std::filesystem::exists(exportFile));
  std::string exportContent = readFile(exportFile);

  // Should have header
  CHECK(exportContent.find("uid") != std::string::npos);
  CHECK(exportContent.find("timestamp_in") != std::string::npos);
  CHECK(exportContent.find("timestamp_out") != std::string::npos);

  // Should have data for both users
  CHECK(exportContent.find("1") != std::string::npos);
  CHECK(exportContent.find("2") != std::string::npos);

  // Clean up
  std::filesystem::remove(importFile);
  std::filesystem::remove(exportFile);
}

TEST_CASE("TrajectoryCollection - Import malformed CSV") {
  std::string testFile = "test_malformed.csv";

  // Create CSV with missing columns
  std::string csvContent =
      "uid;timestamp\n"
      "1;1000\n";
  createTestCSV(testFile, csvContent);

  TrajectoryCollection collection;

  // Should throw or handle gracefully
  CHECK_THROWS(collection.import(testFile));

  // Clean up
  std::filesystem::remove(testFile);
}
