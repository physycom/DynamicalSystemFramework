
#pragma once

#include <concepts>
#include <memory>
#include <type_traits>

namespace dsf {
  class Node;

  namespace mobility {
    class Intersection;

    class TrafficLight;

    class Roundabout;
  }  // namespace mobility

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
  struct is_node<mobility::Intersection> : std::true_type {};

  template <>
  struct is_node<const mobility::Intersection> : std::true_type {};

  template <>
  struct is_node<const mobility::Intersection&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<mobility::Intersection>> : std::true_type {};

  template <>
  struct is_node<mobility::TrafficLight> : std::true_type {};

  template <>
  struct is_node<const mobility::TrafficLight> : std::true_type {};

  template <>
  struct is_node<const mobility::TrafficLight&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<mobility::TrafficLight>> : std::true_type {};

  template <>
  struct is_node<mobility::Roundabout> : std::true_type {};

  template <>
  struct is_node<const mobility::Roundabout> : std::true_type {};

  template <>
  struct is_node<const mobility::Roundabout&> : std::true_type {};

  template <>
  struct is_node<std::unique_ptr<mobility::Roundabout>> : std::true_type {};

  template <typename T>
  inline constexpr bool is_node_v = is_node<T>::value;

};  // namespace dsf
