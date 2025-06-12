#include <cassert>
#include <cstdint>

#include "is_numeric.hpp"

using dsf::is_numeric;
using dsf::is_numeric_v;

static_assert(is_numeric<int8_t>::value);
static_assert(is_numeric<uint8_t>::value);
static_assert(is_numeric<int16_t>::value);
static_assert(is_numeric<uint16_t>::value);
static_assert(is_numeric<int32_t>::value);
static_assert(is_numeric<uint32_t>::value);
static_assert(is_numeric<float>::value);
static_assert(is_numeric<double>::value);
static_assert(is_numeric<long double>::value);
static_assert(!is_numeric<char>::value);
static_assert(!is_numeric<bool>::value);

static_assert(is_numeric_v<int8_t>);
static_assert(is_numeric_v<uint8_t>);
static_assert(is_numeric_v<int16_t>);
static_assert(is_numeric_v<uint16_t>);
static_assert(is_numeric_v<int32_t>);
static_assert(is_numeric_v<uint32_t>);
static_assert(is_numeric_v<float>);
static_assert(is_numeric_v<double>);
static_assert(is_numeric_v<long double>);
static_assert(!is_numeric_v<char>);
static_assert(!is_numeric_v<bool>);
