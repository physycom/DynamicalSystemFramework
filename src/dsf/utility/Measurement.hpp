#pragma once

#include <algorithm>
#include <cmath>

namespace dsf {
  /// @brief The Measurement struct represents the mean of a quantity and its standard deviation
  /// @tparam T The type of the quantity
  /// @param mean The mean
  /// @param std The standard deviation of the sample
  /// @param is_valid True if the measurement is valid, false otherwise (i.e. checks if the sample is not empty)
  template <typename T>
  struct Measurement {
    T mean = static_cast<T>(0);
    T std = static_cast<T>(0);
    bool is_valid = false;

    Measurement(T mean, T std) : mean{mean}, std{std}, is_valid{true} {}
    template <typename TContainer>
    Measurement(TContainer const& data) {
      if (data.empty()) {
        return;
      }
      is_valid = true;
      auto x_mean = static_cast<T>(0), x2_mean = static_cast<T>(0);

      std::for_each(data.begin(), data.end(), [&x_mean, &x2_mean](auto value) -> void {
        x_mean += value;
        x2_mean += value * value;
      });
      mean = x_mean / data.size();
      std = std::sqrt(x2_mean / data.size() - mean * mean);
    }
  };
}  // namespace dsf