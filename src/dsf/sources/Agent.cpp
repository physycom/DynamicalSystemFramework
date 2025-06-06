#include "../headers/Agent.hpp"

namespace dsf {
  Agent::Agent(Time const& spawnTime,
               std::optional<Id> itineraryId,
               std::optional<Id> srcNodeId)
      : m_spawnTime{spawnTime},
        m_freeTime{0},
        m_id{0},
        m_trip{itineraryId.has_value() ? std::vector<Id>{*itineraryId}
                                       : std::vector<Id>{}},
        m_srcNodeId{srcNodeId},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}
  Agent::Agent(Time const& spawnTime,
               std::vector<Id> const& trip,
               std::optional<Id> srcNodeId)
      : m_spawnTime{spawnTime},
        m_freeTime{spawnTime},
        m_id{0},
        m_trip{trip},
        m_srcNodeId{srcNodeId},
        m_nextStreetId{std::nullopt},
        m_itineraryIdx{0},
        m_speed{0.},
        m_distance{0.} {}

  void Agent::setSrcNodeId(Id srcNodeId) { m_srcNodeId = srcNodeId; }
  void Agent::setStreetId(std::optional<Id> streetId) {
    if (!streetId.has_value()) {
      assert(m_nextStreetId.has_value());
      m_streetId = std::move(m_nextStreetId);
      return;
    }
    assert(m_nextStreetId.has_value() ? streetId == m_nextStreetId.value() : true);
    m_streetId = streetId;
    m_nextStreetId = std::nullopt;
  }
  void Agent::setNextStreetId(Id nextStreetId) { m_nextStreetId = nextStreetId; }
  void Agent::setSpeed(double speed) {
    if (speed < 0.) {
      Logger::error(std::format("Speed ({}) of agent {} must be positive", speed, m_id));
    }
    m_speed = speed;
  }
  void Agent::setFreeTime(Time const& freeTime) { m_freeTime = freeTime; }

  void Agent::incrementDistance(double distance) {
    if (distance < 0) {
      Logger::error(std::format(
          "Distance travelled ({}) by agent {} must be positive", distance, m_id));
    }
    m_distance += distance;
  }
  void Agent::updateItinerary() {
    if (m_itineraryIdx < m_trip.size() - 1) {
      ++m_itineraryIdx;
    }
  }
  void Agent::reset(Time const& spawnTime) {
    m_spawnTime = spawnTime;
    m_freeTime = 0;
    m_streetId = std::nullopt;
    m_speed = 0.;
    m_distance = 0.;
    m_itineraryIdx = 0;
  }

  Time const& Agent::spawnTime() const { return m_spawnTime; }
  Time const& Agent::freeTime() const { return m_freeTime; }
  Id Agent::id() const { return m_id; }
  Id Agent::itineraryId() const {
    assert(m_itineraryIdx < m_trip.size());
    return m_trip[m_itineraryIdx];
  }
  std::vector<Id> const& Agent::trip() const { return m_trip; }
  std::optional<Id> Agent::streetId() const { return m_streetId; }
  std::optional<Id> Agent::srcNodeId() const { return m_srcNodeId; }
  std::optional<Id> Agent::nextStreetId() const { return m_nextStreetId; }
  double Agent::speed() const { return m_speed; }
  double Agent::distance() const { return m_distance; }
  bool Agent::isRandom() const { return m_trip.empty(); }
}  // namespace dsf
