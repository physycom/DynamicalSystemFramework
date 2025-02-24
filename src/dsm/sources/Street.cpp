
#include "../headers/Street.hpp"

#include <algorithm>
#include <cassert>

namespace dsm {
  Street::Street(Id id, Street&& street)
      : Road(id,
             street.nodePair(),
             street.length(),
             street.maxSpeed(),
             street.nLanes(),
             street.name(),
             street.geometry(),
             street.capacity(),
             street.transportCapacity()),
             m_movingAgents{std::priority_queue<std::unique_ptr<Agent>>()} {
    for (auto i{0}; i < street.nLanes(); ++i) {
      m_exitQueues.push_back(dsm::queue<std::unique_ptr<Agent>>());
    }
    m_laneMapping = street.laneMapping();
  }

  Street::Street(Id id,
                 std::pair<Id, Id> nodePair,
                 double length,
                 double maxSpeed,
                 int nLanes,
                 std::string name,
                 std::vector<std::pair<double, double>> geometry,
                 std::optional<int> capacity,
                 int transportCapacity)
      : Road(id,
             std::move(nodePair),
             length,
             maxSpeed,
             nLanes,
             std::move(name),
             std::move(geometry),
             capacity,
             transportCapacity),
             m_movingAgents{std::priority_queue<std::unique_ptr<Agent>>()} {
    m_exitQueues.resize(nLanes);
    for (auto i{0}; i < nLanes; ++i) {
      m_exitQueues.push_back(dsm::queue<std::unique_ptr<Agent>>());
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

  void Street::addAgent(std::unique_ptr<Agent> pAgent) {
    assert(!isFull());
    m_movingAgents.push(std::move(pAgent));
  }
  void Street::enqueue(size_t const& queueId) {
    assert(!m_movingAgents.empty());
    m_exitQueues[queueId].push(m_movingAgents.top());
    m_movingAgents.pop();
  }
  std::unique_ptr<Agent> Street::dequeue(size_t index) {
    assert(!m_exitQueues[index].empty());
    auto pAgent{std::move(m_exitQueues[index].front())};
    m_exitQueues[index].pop();
    return pAgent;
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
                                     std::vector<std::pair<double, double>> geometry,
                                     double flowRate,
                                     std::optional<int> capacity,
                                     int transportCapacity)
      : Street(id,
               std::move(nodePair),
               length,
               maxSpeed,
               nLanes,
               std::move(name),
               std::move(geometry),
               capacity,
               transportCapacity) {
    setFlowRate(flowRate);
  }
  void StochasticStreet::setFlowRate(double const flowRate) {
    if (flowRate < 0. || flowRate > 1.) {
      Logger::error(std::format("Flow rate ({}) must be in [0, 1]", flowRate));
    }
    m_flowRate = flowRate;
  }
  double StochasticStreet::flowRate() const { return m_flowRate; }
  bool StochasticStreet::isStochastic() const { return true; }

  void SpireStreet::addAgent(std::unique_ptr<Agent> pAgent) {
    Street::addAgent(std::move(pAgent));
    increaseInputCounter();
  }

  int SpireStreet::meanFlow() { return inputCounts() - outputCounts(); }

  std::unique_ptr<Agent> SpireStreet::dequeue(size_t index) {
    increaseOutputCounter();
    return Street::dequeue(index);
  }
  void StochasticSpireStreet::addAgent(std::unique_ptr<Agent> pAgent) {
    Street::addAgent(std::move(pAgent));
    increaseInputCounter();
  }

  int StochasticSpireStreet::meanFlow() { return inputCounts() - outputCounts(); }

  std::unique_ptr<Agent> StochasticSpireStreet::dequeue(size_t index) {
    increaseOutputCounter();
    return Street::dequeue(index);
  }
};  // namespace dsm
