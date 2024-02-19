#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

std::vector<long> parse_massif(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + file_path);
  }

  std::vector<long> mem_values;

  const char equal{'='};
  std::string buffer;
  while (getline(file, buffer)) {
    if (buffer.find("mem_heap_B") != std::string::npos) {
      std::stringstream buffer_stream(buffer);
      std::string value;
      getline(buffer_stream, value, equal);
      if (value == "mem_heap_B") {
        getline(buffer_stream, value);
        mem_values.push_back(std::stol(value));
      }
    }
  }

  file.close();
  return mem_values;
};

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <massif_file>\n";
    return 1;
  }

  auto mem_values{parse_massif(argv[1])};
  long long int integral{std::accumulate(mem_values.begin(), mem_values.end(), 0)};
  long max_mem{*std::max_element(mem_values.begin(), mem_values.end())};

  std::cout << "integral: " << integral << " B\n";
  std::cout << "max_mem: " << max_mem << " B\n";
}
