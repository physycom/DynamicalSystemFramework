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
#include <chrono>
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
#include <iomanip>
#ifdef __APPLE__
#include <sstream>
#endif

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
    std::string m_name;
    std::time_t m_timeInit, m_timeStep;

  protected:
    tbb::task_arena m_taskArena;
    std::mt19937_64 m_generator;

  protected:
    inline void m_evolve() { ++m_timeStep; };

    /// @brief Get a safe date-time string for filenames (YYYYMMDD_HHMMSS)
    /// @return std::string, The safe date-time string
    inline auto m_safeDateTime() const {
#ifdef __APPLE__
      std::time_t const t = time();
      std::ostringstream oss;
      oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
      return oss.str();
#else
      return std::format(
          "{:%Y%m%d_%H%M%S}",
          std::chrono::floor<std::chrono::seconds>(std::chrono::current_zone()->to_local(
              std::chrono::system_clock::from_time_t(time()))));
#endif
    }
    /// @brief Get a safe name string for filenames (spaces replaced by underscores)
    /// @return std::string, The safe name string
    inline auto m_safeName() const {
      std::string safeName = m_name;
      std::replace(safeName.begin(), safeName.end(), ' ', '_');
      return safeName;
    }

  public:
    /// @brief Construct a new Dynamics object
    /// @param graph The graph representing the network
    /// @param seed The seed for the random number generator (default is std::nullopt)
    Dynamics(network_t& graph, std::optional<unsigned int> seed = std::nullopt);

    /// @brief Set the name of the simulation
    /// @param name The name of the simulation
    inline void setName(const std::string& name) { m_name = name; };
    /// @brief Set the initial time as epoch time
    /// @param timeEpoch The initial time as epoch time
    inline void setInitTime(std::time_t timeEpoch) { m_timeInit = timeEpoch; };

    /// @brief Get the graph
    /// @return const network_t&, The graph
    inline const auto& graph() const { return m_graph; };
    /// @brief Get the name of the simulation
    /// @return const std::string&, The name of the simulation
    inline const auto& name() const { return m_name; };
    /// @brief Get the current simulation time as epoch time
    /// @return std::time_t, The current simulation time as epoch time
    inline auto time() const { return m_timeInit + m_timeStep; }
    /// @brief Get the current simulation time-step
    /// @return std::time_t, The current simulation time-step
    inline auto time_step() const { return m_timeStep; }
    /// @brief Get the current simulation time as formatted string (YYYY-MM-DD HH:MM:SS)
    /// @return std::string, The current simulation time as formatted string
    inline auto strDateTime() const {
#ifdef __APPLE__
      std::time_t const t = time();
      std::ostringstream oss;
      oss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
      return oss.str();
#else
      return std::format(
          "{:%Y-%m-%d %H:%M:%S}",
          std::chrono::floor<std::chrono::seconds>(std::chrono::current_zone()->to_local(
              std::chrono::system_clock::from_time_t(time()))));
#endif
    }
  };

  template <typename network_t>
  Dynamics<network_t>::Dynamics(network_t& graph, std::optional<unsigned int> seed)
      : m_graph{std::move(graph)},
        m_name{"unnamed simulation"},
        m_timeInit{0},
        m_timeStep{0},
        m_generator{std::random_device{}()} {
    if (seed.has_value()) {
      m_generator.seed(*seed);
    }
    m_taskArena.initialize();
  }
};  // namespace dsf
