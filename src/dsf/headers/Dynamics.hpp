/// @file       /src/dsf/headers/Dynamics.hpp
/// @brief      Defines the Dynamics class.
///
/// @details    This file contains the definition of the Dynamics class.
///             The Dynamics class represents the dynamics of the network. It is templated by the type
///             of the graph's id and the type of the graph's capacity.
///             The graph's id and capacity must be unsigned integral types.

#pragma once

#include "Network.hpp"
#include "../utility/Typedef.hpp"

#include <algorithm>
#include <cassert>
#include <concepts>
#include <vector>
#include <random>
#include <span>
#include <numeric>
#include <unordered_map>
#include <cmath>
#include <cassert>
#include <format>
#include <thread>
#include <exception>
#include <fstream>
#include <filesystem>
#include <functional>

#include <tbb/tbb.h>

namespace dsf {
  /// @brief The Measurement struct represents the mean of a quantity and its standard deviation
  /// @tparam T The type of the quantity
  /// @param mean The mean
  /// @param std The standard deviation of the sample
  template <typename T>
  struct Measurement {
    T mean;
    T std;

    Measurement(T mean, T std) : mean{mean}, std{std} {}
    Measurement(std::span<T> data) {
      auto x_mean = static_cast<T>(0), x2_mean = static_cast<T>(0);
      if (data.empty()) {
        mean = static_cast<T>(0);
        std = static_cast<T>(0);
        return;
      }

      std::for_each(data.begin(), data.end(), [&x_mean, &x2_mean](auto value) -> void {
        x_mean += value;
        x2_mean += value * value;
      });
      mean = x_mean / data.size();
      std = std::sqrt(x2_mean / data.size() - mean * mean);
    }
  };

  /// @brief The Dynamics class represents the dynamics of the network.
  /// @tparam network_t The type of the network
  template <typename network_t>
  class Dynamics {
  private:
    network_t m_graph;

  protected:
    tbb::task_arena m_taskArena;
    Time m_time;
    std::mt19937_64 m_generator;

  protected:
    void m_evolve() { ++m_time; };

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    Dynamics(network_t& graph, std::optional<unsigned int> seed = std::nullopt);

    /// @brief Reset the simulation time to 0
    void resetTime() { m_time = 0; };

    /// @brief Get the graph
    /// @return const network_t&, The graph
    const network_t& graph() const { return m_graph; };
    /// @brief Get the current simulation time-step
    /// @return Time The current simulation time
    Time time() const { return m_time; }
  };

  template <typename network_t>
  Dynamics<network_t>::Dynamics(network_t& graph, std::optional<unsigned int> seed)
      : m_graph{std::move(graph)}, m_time{0}, m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(*seed);
    }
    m_taskArena.initialize();
  }
};  // namespace dsf
