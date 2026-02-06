#include "FirstOrderDynamics.hpp"

namespace dsf::mobility {
  double FirstOrderDynamics::m_speedFactor(double const& density) const {
    return (1. - m_alpha * density);
  }
  double FirstOrderDynamics::m_streetEstimatedTravelTime(
      std::unique_ptr<Street> const& pStreet) const {
    return pStreet->length() /
           (pStreet->maxSpeed() * m_speedFactor(pStreet->density(true)));
  }
  void FirstOrderDynamics::m_dumpSimInfo() const {
    // Dump simulation info (parameters) to the database, if connected
    if (!this->database()) {
      return;
    }
    // Create simulations table if it doesn't exist
    SQLite::Statement createTableStmt(*this->database(),
                                      "CREATE TABLE IF NOT EXISTS simulations ("
                                      "id INTEGER PRIMARY KEY, "
                                      "name TEXT, "
                                      "alpha REAL, "
                                      "speed_fluctuation_std REAL, "
                                      "weight_function TEXT, "
                                      "weight_threshold REAL NOT NULL, "
                                      "error_probability REAL, "
                                      "passage_probability REAL, "
                                      "mean_travel_distance_m REAL, "
                                      "mean_travel_time_s REAL, "
                                      "stagnant_tolerance_factor REAL, "
                                      "force_priorities BOOLEAN, "
                                      "save_avg_stats BOOLEAN, "
                                      "save_road_data BOOLEAN, "
                                      "save_travel_data BOOLEAN)");
    createTableStmt.exec();
    // Insert simulation parameters into the simulations table
    SQLite::Statement insertSimStmt(
        *this->database(),
        "INSERT INTO simulations (id, name, alpha, speed_fluctuation_std, "
        "weight_function, weight_threshold, error_probability, passage_probability, "
        "mean_travel_distance_m, mean_travel_time_s, stagnant_tolerance_factor, "
        "force_priorities, save_avg_stats, save_road_data, save_travel_data) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insertSimStmt.bind(1, static_cast<std::int64_t>(this->id()));
    insertSimStmt.bind(2, this->name());
    insertSimStmt.bind(3, m_alpha);
    insertSimStmt.bind(4, m_speedFluctuationSTD);
    switch (this->m_pathWeight) {
      case PathWeight::LENGTH:
        insertSimStmt.bind(5, "LENGTH");
        break;
      case PathWeight::TRAVELTIME:
        insertSimStmt.bind(5, "TRAVELTIME");
        break;
      case PathWeight::WEIGHT:
        insertSimStmt.bind(5, "WEIGHT");
        break;
    }
    insertSimStmt.bind(6, this->m_weightTreshold);
    if (this->m_errorProbability.has_value()) {
      insertSimStmt.bind(7, *this->m_errorProbability);
    } else {
      insertSimStmt.bind(7);
    }
    if (this->m_passageProbability.has_value()) {
      insertSimStmt.bind(8, *this->m_passageProbability);
    } else {
      insertSimStmt.bind(8);
    }
    if (this->m_meanTravelDistance.has_value()) {
      insertSimStmt.bind(9, *this->m_meanTravelDistance);
    } else {
      insertSimStmt.bind(9);
    }
    if (this->m_meanTravelTime.has_value()) {
      insertSimStmt.bind(10, static_cast<int64_t>(*this->m_meanTravelTime));
    } else {
      insertSimStmt.bind(10);
    }
    if (this->m_timeToleranceFactor.has_value()) {
      insertSimStmt.bind(11, *this->m_timeToleranceFactor);
    } else {
      insertSimStmt.bind(11);
    }
    insertSimStmt.bind(12, this->m_forcePriorities);
    insertSimStmt.bind(13, this->m_bSaveAverageStats);
    insertSimStmt.bind(14, this->m_bSaveStreetData);
    insertSimStmt.bind(15, this->m_bSaveTravelData);
    insertSimStmt.exec();
  }
  FirstOrderDynamics::FirstOrderDynamics(RoadNetwork& graph,
                                         bool useCache,
                                         std::optional<unsigned int> seed,
                                         double alpha,
                                         PathWeight const weightFunction,
                                         std::optional<double> weightTreshold)
      : RoadDynamics<Delay>(graph, useCache, seed, weightFunction, weightTreshold),
        m_alpha{alpha},
        m_speedFluctuationSTD{0.} {
    if (alpha < 0. || alpha > 1.) {
      throw std::invalid_argument(
          std::format("The minimum speed ratio ({}) must be in [0, 1[", alpha));
    }
    double globMaxTimePenalty{0.};
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      globMaxTimePenalty =
          std::max(globMaxTimePenalty,
                   std::ceil(pStreet->length() / ((1. - m_alpha) * pStreet->maxSpeed())));
    }
    if (globMaxTimePenalty > static_cast<double>(std::numeric_limits<Delay>::max())) {
      throw std::overflow_error(
          std::format("The maximum time penalty ({}) is greater than the "
                      "maximum value of delay_t ({})",
                      globMaxTimePenalty,
                      std::numeric_limits<Delay>::max()));
    }
  }

  void FirstOrderDynamics::setAgentSpeed(std::unique_ptr<Agent> const& pAgent) {
    const auto& street{this->graph().edge(pAgent->streetId().value())};
    double speed{street->maxSpeed() * this->m_speedFactor(street->density(true))};
    if (m_speedFluctuationSTD > 0.) {
      std::normal_distribution<double> speedDist{speed, speed * m_speedFluctuationSTD};
      speed = speedDist(this->m_generator);
    }
    speed < 0. ? pAgent->setSpeed(street->maxSpeed() * (1. - m_alpha))
               : pAgent->setSpeed(speed);
  }

  void FirstOrderDynamics::setSpeedFluctuationSTD(double speedFluctuationSTD) {
    if (speedFluctuationSTD < 0.) {
      throw std::invalid_argument(
          std::format("The speed fluctuation standard deviation ({}) must be positive",
                      speedFluctuationSTD));
    }
    m_speedFluctuationSTD = speedFluctuationSTD;
  }
}  // namespace dsf::mobility