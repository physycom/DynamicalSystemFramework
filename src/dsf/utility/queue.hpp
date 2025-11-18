#pragma once

#include <queue>

namespace dsf {
  // use the default allocator for std::deque
  template <typename T, typename Container = std::deque<T>>
  class queue : public std::queue<T, Container> {
  public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    // c is a protected member of std::queue, which is the underlying container
    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }

    T& operator[](size_t i) { return *(this->c.begin() + i); }
    const T& operator[](size_t i) const { return *(this->c.begin() + i); }
  };

  template <typename T,
            typename Container = std::vector<T>,
            typename Compare = std::less<typename Container::value_type>>
  class priority_queue : public std::priority_queue<T, Container, Compare> {
  public:
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    // c is a protected member of std::queue, which is the underlying container
    iterator begin() { return this->c.begin(); }
    iterator end() { return this->c.end(); }
    const_iterator begin() const { return this->c.begin(); }
    const_iterator end() const { return this->c.end(); }

    T& operator[](size_t i) { return *(this->c.begin() + i); }
    const T& operator[](size_t i) const { return *(this->c.begin() + i); }
  };

};  // namespace dsf
