
#include "../headers/Street.hpp"

#include <algorithm>
#include <cassert>

namespace dsm {
  Street::Street(Id id,
                 std::pair<Id, Id> nodePair,
                 double length,
                 double maxSpeed,
                 int nLanes,
                 std::string name,
                 std::vector<std::pair<double, double>> geometry,
                 std::optional<int> capacity,
                 double transportCapacity)
      : Road(id,
             std::move(nodePair),
             length,
             maxSpeed,
             nLanes,
             std::move(name),
             std::move(geometry),
             capacity,
             transportCapacity),
        m_exitQueues{std::vector<dsm::queue<std::unique_ptr<Agent>>>(nLanes)},
        m_movingAgents{dsm::priority_queue<std::unique_ptr<Agent>,
                                           std::vector<std::unique_ptr<Agent>>,
                                           AgentComparator>()} {
    switch (nLanes) {
      case 1:
        m_laneMapping.emplace_back(Direction::ANY);
        break;
      case 2:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
        m_laneMapping.emplace_back(Direction::LEFT);
        break;
      case 3:
        m_laneMapping.emplace_back(Direction::RIGHTANDSTRAIGHT);
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

  void Street::setLaneMapping(std::vector<Direction> const& laneMapping) {
    assert(laneMapping.size() == static_cast<size_t>(m_nLanes));
    m_laneMapping = laneMapping;
    std::string strLaneMapping;
    std::for_each(
        laneMapping.cbegin(), laneMapping.cend(), [&strLaneMapping](auto const item) {
          strLaneMapping += std::format("{} - ", static_cast<int>(item));
        });
    Logger::debug(std::format("New lane mapping for street {} -> {} is: {}",
                              m_nodePair.first,
                              m_nodePair.second,
                              strLaneMapping));
  }
  void Street::setQueue(dsm::queue<std::unique_ptr<Agent>> queue, size_t index) {
    assert(index < m_exitQueues.size());
    m_exitQueues[index] = std::move(queue);
  }

  void Street::addAgent(std::unique_ptr<Agent> pAgent) {
    assert(!isFull());
    m_movingAgents.push(std::move(pAgent));
  }
  void Street::enqueue(size_t const& queueId) {
    assert(!m_movingAgents.empty());
    m_movingAgents.top()->incrementDistance(m_length);
    Logger::debug("Pusing into queue");
    m_exitQueues[queueId].push(
        std::move(const_cast<std::unique_ptr<Agent>&>(m_movingAgents.top())));
    m_movingAgents.pop();
    Logger::debug("Popped from moving queue");
  }
  std::unique_ptr<Agent> Street::dequeue(size_t index) {
    assert(!m_exitQueues[index].empty());
    Logger::debug("Dequeueing from queue");
    auto pAgent{std::move(m_exitQueues[index].front())};
    m_exitQueues[index].pop();
    return pAgent;
  }

  int Street::nAgents() const {
    auto nAgents{static_cast<int>(m_movingAgents.size())};
    Logger::debug(std::format("Number of moving agents street {}: {}", id(), nAgents));
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    Logger::debug(std::format("Number of agents street {}: {}", id(), nAgents));
    return nAgents;
  }

  double Street::density(bool normalized) const {
    return normalized ? nAgents() / static_cast<double>(m_capacity)
                      : nAgents() / (m_length * m_nLanes);
  }

  int Street::nMovingAgents() const { return m_movingAgents.size(); }
  double Street::nExitingAgents(Direction direction, bool normalizeOnNLanes) const {
    double nAgents{0.};
    int n{0};
    for (auto i{0}; i < m_nLanes; ++i) {
      assert(i < m_exitQueues.size());
      if (direction == Direction::ANY || m_laneMapping[i] == Direction::ANY) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (m_laneMapping[i] == direction) {
        nAgents += m_exitQueues[i].size();
      } else if (m_laneMapping[i] == Direction::RIGHTANDSTRAIGHT &&
                 (direction == Direction::RIGHT || direction == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (m_laneMapping[i] == Direction::LEFTANDSTRAIGHT &&
                 (direction == Direction::LEFT || direction == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (direction == Direction::RIGHTANDSTRAIGHT &&
                 (m_laneMapping[i] == Direction::RIGHT ||
                  m_laneMapping[i] == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      } else if (direction == Direction::LEFTANDSTRAIGHT &&
                 (m_laneMapping[i] == Direction::LEFT ||
                  m_laneMapping[i] == Direction::STRAIGHT)) {
        nAgents += m_exitQueues[i].size();
        ++n;
      }
    }
    if (normalizeOnNLanes && n > 1) {
      nAgents /= static_cast<double>(n);
    }
    return nAgents;
  }

  StochasticStreet::StochasticStreet(Street&& street, double flowRate)
      : Street(std::move(street)) {
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
                                     double transportCapacity)
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
