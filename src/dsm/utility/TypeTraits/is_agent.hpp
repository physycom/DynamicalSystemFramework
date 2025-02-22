
// #pragma once

// #include <concepts>
// #include <memory>
// #include <type_traits>
// #include "is_numeric.hpp"

// namespace dsm {
//   class Agent;

//   // define is_node type trait
//   template <typename T>
//   struct is_agent : std::false_type {};

//   struct is_agent<Agent> : std::true_type {};

//   struct is_agent<std::unique_ptr<Agent>> : std::true_type {};

//   template <typename T>
//   inline constexpr bool is_agent_v = is_agent<T>::value;

// };  // namespace dsm
