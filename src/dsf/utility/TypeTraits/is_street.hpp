
#pragma once

#include <concepts>
#include <memory>
#include <type_traits>

namespace dsf {
  class Street;

  // define the is_street type trait
  template <typename T>
  struct is_street : std::false_type {};

  template <>
  struct is_street<Street> : std::true_type {};

  template <>
  struct is_street<const Street> : std::true_type {};

  template <>
  struct is_street<const Street&> : std::true_type {};

  template <>
  struct is_street<std::unique_ptr<Street>> : std::true_type {};

  template <typename T>
  inline constexpr bool is_street_v = is_street<T>::value;

};  // namespace dsf
