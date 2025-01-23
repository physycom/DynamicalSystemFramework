
#include "Street.hpp"

#include <algorithm>
#include <cassert>

namespace dsm {
  Street::Street(Id id, const Street& street)
      : Road(id,
             street.nodePair(),
             street.length(),
             street.maxSpeed(),
             street.nLanes(),
             street.name(),
             street.capacity(),
             street.transportCapacity()) {
    for (auto i{0}; i < street.nLanes(); ++i) {
      m_exitQueues.push_back(dsm::queue<Size>());
    }
    m_laneMapping = street.laneMapping();
  }

  Street::Street(Id id,
                 std::pair<Id, Id> nodePair,
                 double length,
                 double maxSpeed,
                 int nLanes,
                 std::string name,
                 std::optional<int> capacity,
                 int transportCapacity)
      : Road(id,
             std::move(nodePair),
             length,
             maxSpeed,
             nLanes,
             std::move(name),
             capacity,
             transportCapacity) {
    m_exitQueues.resize(nLanes);
    for (auto i{0}; i < nLanes; ++i) {
      m_exitQueues.push_back(dsm::queue<Size>());
    }
    switch (nLanes) {
      case 1:
        m_laneMapping.emplace_back(Direction::ANY);
        break;
      case 2:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      case 3:
        m_laneMapping.emplace_back(Direction::RIGHT);
        m_laneMapping.emplace_back(Direction::STRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      default:
        m_laneMapping.emplace_back(Direction::RIGHT);
        for (auto i{1}; i < nLanes - 1; ++i) {
          m_laneMapping.emplace_back(Direction::STRAIGHT);
        }
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
    }
  }

  std::vector<Id> const& Street::movingAgents() const { return m_movingAgents; }

  void Street::addAgent(Id agentId) {
    assert((void("Agent is already on the street."),
            std::find(m_movingAgents.cbegin(), m_movingAgents.cend(), agentId) ==
                m_movingAgents.cend()));
    for (auto const& queue : m_exitQueues) {
      for (auto const& id : queue) {
        assert((void("Agent is already in queue."), id != agentId));
      }
    }
    m_movingAgents.push_back(agentId);
    ;
  }
  void Street::enqueue(Id agentId, size_t index) {
    assert((void("Agent is not on the street."),
            std::find(m_movingAgents.cbegin(), m_movingAgents.cend(), agentId) !=
                m_movingAgents.cend()));
    for (auto const& queue : m_exitQueues) {
      for (auto const& id : queue) {
        assert((void("Agent is already in queue."), id != agentId));
      }
    }
    m_movingAgents.erase(
        std::remove(m_movingAgents.begin(), m_movingAgents.end(), agentId),
        m_movingAgents.end());
    m_exitQueues[index].push(agentId);
  }
  std::optional<Id> Street::dequeue(size_t index) {
    if (m_exitQueues[index].empty()) {
      return std::nullopt;
    }
    Id id = m_exitQueues[index].front();
    m_exitQueues[index].pop();
    return id;
  }

  int Street::nAgents() const {
    auto nAgents{static_cast<int>(m_movingAgents.size())};
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    return nAgents;
  }

  double Street::density(bool normalized) const {
    return normalized ? nAgents() / static_cast<double>(m_capacity)
                      : nAgents() / (m_length * m_nLanes);
  }

  int Street::nExitingAgents() const {
    int nAgents{0};
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    return nAgents;
  }

  StochasticStreet::StochasticStreet(Id id, const Street& street, double flowRate)
      : Street(id, street) {
    setFlowRate(flowRate);
  }
  StochasticStreet::StochasticStreet(Id id,
                                     std::pair<Id, Id> nodePair,
                                     double length,
                                     double maxSpeed,
                                     int nLanes,
                                     std::string name,
                                     double flowRate,
                                     std::optional<int> capacity,
                                     int transportCapacity)
      : Street(id,
               std::move(nodePair),
               length,
               maxSpeed,
               nLanes,
               std::move(name),
               capacity,
               transportCapacity) {
    setFlowRate(flowRate);
  }
  void StochasticStreet::setFlowRate(double const flowRate) {
    if (flowRate < 0. || flowRate > 1.) {
      throw std::invalid_argument(
          buildLog(std::format("Flow rate ({}) must be between 0 and 1", flowRate)));
    }
    m_flowRate = flowRate;
  }
  double StochasticStreet::flowRate() const { return m_flowRate; }
  bool StochasticStreet::isStochastic() const { return true; }

  void SpireStreet::addAgent(Id agentId) {
    Street::addAgent(agentId);
    increaseInputCounter();
  }

  int SpireStreet::meanFlow() { return inputCounts() - outputCounts(); }

  std::optional<Id> SpireStreet::dequeue(size_t index) {
    auto const& id = Street::dequeue(index);
    if (id.has_value()) {
      increaseOutputCounter();
    }
    return id;
  }
  void StochasticSpireStreet::addAgent(Id agentId) {
    Street::addAgent(agentId);
    increaseInputCounter();
  }

  int StochasticSpireStreet::meanFlow() { return inputCounts() - outputCounts(); }

  std::optional<Id> StochasticSpireStreet::dequeue(size_t index) {
    auto const& id = Street::dequeue(index);
    if (id.has_value()) {
      increaseOutputCounter();
    }
    return id;
  }
};  // namespace dsm
