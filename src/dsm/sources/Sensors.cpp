#include "../headers/Sensors.hpp"

namespace dsm {
  void Counter::setCode(std::string const& code) { m_code = code; }
  void Counter::increaseInputCounter() { m_counters.first++; }
  void Counter::increaseOutputCounter() { m_counters.second++; }

  std::string const& Counter::code() const { return m_code; }
  int Counter::inputCounts(bool reset) {
    if (reset) {
      int count{0};
      std::swap(count, m_counters.first);
      return count;
    }
    return m_counters.first;
  }
  int Counter::outputCounts(bool reset) {
    if (reset) {
      int count{0};
      std::swap(count, m_counters.second);
      return count;
    }
    return m_counters.second;
  }
}  // namespace dsm