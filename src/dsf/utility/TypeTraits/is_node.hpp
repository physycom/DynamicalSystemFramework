
#pragma once

#include <concepts>
#include <memory>
#include <type_traits>

namespace dsf {
  class Node;

  class Intersection;

  class TrafficLight;

  class Roundabout;

  // define is_node type trait
  template <typename T>
  struct is_node : std::false_type {};

  template <>
  struct is_node<Node> : std::true_type {};

  template <>
  struct is_node<const Node> : std::true_type {};

  template <>
  struct is_node<const Node&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<Node>> : std::true_type {};

  // TODO: this is bad, I'll rework the type-traits
  template <>
  struct is_node<Intersection> : std::true_type {};

  template <>
  struct is_node<const Intersection> : std::true_type {};

  template <>
  struct is_node<const Intersection&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<Intersection>> : std::true_type {};

  template <>
  struct is_node<TrafficLight> : std::true_type {};

  template <>
  struct is_node<const TrafficLight> : std::true_type {};

  template <>
  struct is_node<const TrafficLight&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<TrafficLight>> : std::true_type {};

  template <>
  struct is_node<Roundabout> : std::true_type {};

  template <>
  struct is_node<const Roundabout> : std::true_type {};

  template <>
  struct is_node<const Roundabout&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<Roundabout>> : std::true_type {};

  template <typename T>
  inline constexpr bool is_node_v = is_node<T>::value;

};  // namespace dsf
