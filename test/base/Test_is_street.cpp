#include <cassert>
#include <cstdint>
#include <memory>

#include "is_street.hpp"

using dsf::is_street;
using dsf::is_street_v;
using dsf::mobility::Street;

// check the type trait
static_assert(is_street<Street>::value);
static_assert(!is_street<int>::value);
static_assert(!is_street<unsigned int>::value);
static_assert(!is_street<std::string>::value);
static_assert(is_street<std::unique_ptr<Street>>::value);
static_assert(!is_street<std::unique_ptr<int>>::value);
static_assert(!is_street<std::unique_ptr<unsigned int>>::value);
static_assert(!is_street<std::unique_ptr<std::string>>::value);

// check the template variable
static_assert(is_street_v<Street>);
static_assert(!is_street_v<int>);
static_assert(!is_street_v<unsigned int>);
static_assert(!is_street_v<std::string>);
static_assert(is_street_v<std::unique_ptr<Street>>);
static_assert(!is_street_v<std::unique_ptr<int>>);
static_assert(!is_street_v<std::unique_ptr<unsigned int>>);
static_assert(!is_street_v<std::unique_ptr<std::string>>);
