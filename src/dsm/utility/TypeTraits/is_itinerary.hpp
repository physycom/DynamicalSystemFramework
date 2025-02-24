
// #pragma once

// #include <concepts>
// #include <memory>
// #include <type_traits>

// namespace dsm {
//   class Itinerary;

//   // define is_node type trait
//   template <typename T>
//   struct is_itinerary : std::false_type {};

//   struct is_itinerary<Agent> : std::true_type {};

//   struct is_itinerary<std::unique_ptr<Agent>> : std::true_type {};

//   template <typename T>
//   inline constexpr bool is_itinerary_v = is_itinerary<T>::value;

// };  // namespace dsm
