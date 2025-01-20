
#include "Street.hpp"

namespace dsm {
  double Street::m_meanVehicleLength = 5.;

  Street::Street(Id id, const Street& street)
      : Edge(id,
             street.nodePair(),
             street.capacity(),
             street.transportCapacity(),
             street.angle()),
        m_length{street.length()},
        m_maxSpeed{street.maxSpeed()},
        m_nLanes{street.nLanes()},
        m_name{street.name()} {
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
      : Edge(id,
             std::move(nodePair),
             capacity.value_or(std::ceil((length * nLanes) / m_meanVehicleLength)),
             transportCapacity),
        m_length{length},
        m_maxSpeed{maxSpeed},
        m_nLanes{nLanes},
        m_name{std::move(name)} {
    if (!(length > 0.)) {
      throw std::invalid_argument(buildLog(
          std::format("The length of a street ({}) must be greater than 0.", length)));
    }
    if (!(maxSpeed > 0.)) {
      throw std::invalid_argument(buildLog(std::format(
          "The maximum speed of a street ({}) must be greater than 0.", maxSpeed)));
    }
    if (nLanes < 1) {
      throw std::invalid_argument(buildLog(std::format(
          "The number of lanes of a street ({}) must be greater than 0.", nLanes)));
    }
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

  void Street::setMaxSpeed(double speed) {
    if (speed < 0.) {
      throw std::invalid_argument(buildLog(
          std::format("The maximum speed of a street ({}) cannot be negative.", speed)));
    }
    m_maxSpeed = speed;
  }
  void Street::setMeanVehicleLength(double meanVehicleLength) {
    if (!(meanVehicleLength > 0.)) {
      throw std::invalid_argument(buildLog(std::format(
          "The mean vehicle length ({}) must be greater than 0.", meanVehicleLength)));
    }
    m_meanVehicleLength = meanVehicleLength;
  }

  void Street::addAgent(Id agentId) {
    assert((void("Agent is already on the street."), !m_waitingAgents.contains(agentId)));
    for (auto const& queue : m_exitQueues) {
      for (auto const& id : queue) {
        assert((void("Agent is already in queue."), id != agentId));
      }
    }
    m_waitingAgents.insert(agentId);
    ;
  }
  void Street::enqueue(Id agentId, size_t index) {
    assert((void("Agent is not on the street."), m_waitingAgents.contains(agentId)));
    for (auto const& queue : m_exitQueues) {
      for (auto const& id : queue) {
        assert((void("Agent is already in queue."), id != agentId));
      }
    }
    m_waitingAgents.erase(agentId);
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
    auto nAgents{static_cast<int>(m_waitingAgents.size())};
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    return nAgents;
  }

  double Street::density(bool normalized) const {
    return normalized ? nAgents() / static_cast<double>(m_capacity)
                      : nAgents() / (m_length * m_nLanes);
  }

  Size Street::nExitingAgents() const {
    Size nAgents{0};
    for (const auto& queue : m_exitQueues) {
      nAgents += queue.size();
    }
    return nAgents;
  }

  void SpireStreet::addAgent(Id agentId) {
    Street::addAgent(agentId);
    ++m_agentCounterIn;
  }

  Size SpireStreet::inputCounts(bool resetValue) {
    if (!resetValue)
      return m_agentCounterIn;
    Size flow = m_agentCounterIn;
    m_agentCounterIn = 0;
    m_agentCounterOut = 0;
    return flow;
  }
  Size SpireStreet::outputCounts(bool resetValue) {
    if (!resetValue)
      return m_agentCounterOut;
    Size flow = m_agentCounterOut;
    m_agentCounterIn = 0;
    m_agentCounterOut = 0;
    return flow;
  }
  int SpireStreet::meanFlow() {
    int flow = static_cast<int>(m_agentCounterIn) - static_cast<int>(m_agentCounterOut);
    m_agentCounterIn = 0;
    m_agentCounterOut = 0;
    return flow;
  }

  std::optional<Id> SpireStreet::dequeue(size_t index) {
    std::optional<Id> id = Street::dequeue(index);
    if (id.has_value()) {
      ++m_agentCounterOut;
    }
    return id;
  }

};  // namespace dsm
