#include <cassert>
#include <cstdint>
#include <memory>

#include "is_node.hpp"

using dsf::is_node;
using dsf::is_node_v;
using dsf::Node;

// check the type trait
static_assert(is_node<Node>::value);
static_assert(!is_node<int>::value);
static_assert(!is_node<unsigned int>::value);
static_assert(!is_node<std::string>::value);
static_assert(is_node<std::unique_ptr<Node>>::value);
static_assert(!is_node<std::unique_ptr<int>>::value);
static_assert(!is_node<std::unique_ptr<unsigned int>>::value);
static_assert(!is_node<std::unique_ptr<std::string>>::value);

// check the template variable
static_assert(is_node_v<Node>);
static_assert(!is_node_v<int>);
static_assert(!is_node_v<unsigned int>);
static_assert(!is_node_v<std::string>);
static_assert(is_node_v<std::unique_ptr<Node>>);
static_assert(!is_node_v<std::unique_ptr<int>>);
static_assert(!is_node_v<std::unique_ptr<unsigned int>>);
static_assert(!is_node_v<std::unique_ptr<std::string>>);
