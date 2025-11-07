
#include "Street.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  Street::Street(Id id,
                 std::pair<Id, Id> nodePair,
                 double length,
                 double maxSpeed,
                 int nLanes,
                 std::string name,
                 geometry::PolyLine geometry,
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
        m_exitQueues{std::vector<dsf::queue<std::unique_ptr<Agent>>>(nLanes)},
        m_movingAgents{dsf::priority_queue<std::unique_ptr<Agent>,
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
  auto Street::operator==(Street const& other) const -> bool {
    bool isEqual{true};
    isEqual &= (this->m_id == other.m_id);
    isEqual &= (this->m_nodePair == other.m_nodePair);
    isEqual &= (this->m_length == other.m_length);
    isEqual &= (this->m_maxSpeed == other.m_maxSpeed);
    isEqual &= (this->m_nLanes == other.m_nLanes);
    isEqual &= (this->m_name == other.m_name);
    isEqual &= (this->m_priority == other.m_priority);
    return isEqual;
  }

  void Street::setLaneMapping(std::vector<Direction> const& laneMapping) {
    assert(laneMapping.size() == static_cast<size_t>(m_nLanes));
    m_laneMapping = laneMapping;
    std::string strLaneMapping;
    std::for_each(
        laneMapping.cbegin(), laneMapping.cend(), [&strLaneMapping](auto const item) {
          strLaneMapping +=
              std::format("{} - ", directionToString[static_cast<size_t>(item)]);
        });
    spdlog::debug("New lane mapping for street {} -> {} is: {}",
                  m_nodePair.first,
                  m_nodePair.second,
                  strLaneMapping);
  }
  void Street::setQueue(dsf::queue<std::unique_ptr<Agent>> queue, size_t index) {
    assert(index < m_exitQueues.size());
    m_exitQueues[index] = std::move(queue);
  }
  void Street::enableCounter(std::string name) {
    if (m_counter.has_value()) {
      throw std::runtime_error(
          std::format("{} already has a counter named {}", *this, m_counter->name()));
    }
    if (name.empty()) {
      name = std::format("Coil_{}", m_id);
    }
    m_counter.emplace();
    m_counter->setName(name);
  }
  void Street::resetCounter() {
    if (!hasCoil()) {
      throw std::runtime_error(
          std::format("Cannot reset counter for {} which does not have a coil.", *this));
    }
    m_counter->reset();
  }

  void Street::addAgent(std::unique_ptr<Agent> pAgent) {
    assert(!isFull());
    spdlog::debug("Adding {} on {}", *pAgent, *this);
    m_movingAgents.push(std::move(pAgent));
  }
  void Street::enqueue(size_t const& queueId) {
    assert(!m_movingAgents.empty());
    m_movingAgents.top()->incrementDistance(m_length);
    m_exitQueues[queueId].push(
        std::move(const_cast<std::unique_ptr<Agent>&>(m_movingAgents.top())));
    m_movingAgents.pop();
    if (m_counter.has_value()) {
      ++(*m_counter);
    }
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

  int Street::nMovingAgents() const { return m_movingAgents.size(); }
  double Street::nExitingAgents(Direction direction, bool normalizeOnNLanes) const {
    double nAgents{0.};
    int n{0};
    for (auto i{0}; i < m_nLanes; ++i) {
      if (direction == Direction::ANY) {
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
    if (normalizeOnNLanes) {
      n > 1 ? nAgents /= n : nAgents;
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
                                     geometry::PolyLine geometry,
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
      throw std::invalid_argument(
          std::format("Flow rate ({}) must be in [0, 1]", flowRate));
    }
    m_flowRate = flowRate;
  }

};  // namespace dsf::mobility
