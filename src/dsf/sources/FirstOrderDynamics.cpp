#include "../headers/FirstOrderDynamics.hpp"

namespace dsf {
  FirstOrderDynamics::FirstOrderDynamics(
      RoadNetwork& graph,
      bool useCache,
      std::optional<unsigned int> seed,
      double alpha,
      std::function<double(const RoadNetwork*, Id, Id)> weightFunction,
      double weightTreshold)
      : RoadDynamics<Delay>(graph, useCache, seed, weightFunction, weightTreshold),
        m_alpha{alpha},
        m_speedFluctuationSTD{0.} {
    if (alpha < 0. || alpha > 1.) {
      Logger::error(std::format("The minimum speed rateo ({}) must be in [0, 1[", alpha));
    }
    double globMaxTimePenalty{0.};
    for (const auto& [streetId, street] : this->graph().edges()) {
      globMaxTimePenalty =
          std::max(globMaxTimePenalty,
                   std::ceil(street->length() / ((1. - m_alpha) * street->maxSpeed())));
    }
    if (globMaxTimePenalty > static_cast<double>(std::numeric_limits<Delay>::max())) {
      throw std::overflow_error(Logger::buildExceptionMessage(
          std::format("The maximum time penalty ({}) is greater than the "
                      "maximum value of delay_t ({})",
                      globMaxTimePenalty,
                      std::numeric_limits<Delay>::max())));
    }
  }

  void FirstOrderDynamics::setAgentSpeed(std::unique_ptr<Agent> const& pAgent) {
    const auto& street{this->graph().edge(pAgent->streetId().value())};
    double speed{street->maxSpeed() * (1. - m_alpha * street->density(true))};
    if (m_speedFluctuationSTD > 0.) {
      std::normal_distribution<double> speedDist{speed, speed * m_speedFluctuationSTD};
      speed = speedDist(this->m_generator);
    }
    speed < 0. ? pAgent->setSpeed(street->maxSpeed() * (1. - m_alpha))
               : pAgent->setSpeed(speed);
  }

  void FirstOrderDynamics::setSpeedFluctuationSTD(double speedFluctuationSTD) {
    if (speedFluctuationSTD < 0.) {
      Logger::error(
          std::format("The speed fluctuation standard deviation ({}) must be positive",
                      speedFluctuationSTD));
    }
    m_speedFluctuationSTD = speedFluctuationSTD;
  }

  double FirstOrderDynamics::streetMeanSpeed(Id streetId) const {
    const auto& street{this->graph().edge(streetId)};
    if (street->nAgents() == 0) {
      return street->maxSpeed();
    }
    double meanSpeed{0.};
    Size n{0};
    if (street->nExitingAgents() == 0) {
      n = static_cast<Size>(street->movingAgents().size());
      double alpha{m_alpha / street->capacity()};
      meanSpeed = street->maxSpeed() * n * (1. - 0.5 * alpha * (n - 1.));
    } else {
      for (auto const& pAgent : street->movingAgents()) {
        meanSpeed += pAgent->speed();
        ++n;
      }
      for (auto const& queue : street->exitQueues()) {
        for (auto const& pAgent : queue) {
          meanSpeed += pAgent->speed();
          ++n;
        }
      }
    }
    const auto& node = this->graph().node(street->nodePair().second);
    if (node->isIntersection()) {
      auto& intersection = dynamic_cast<Intersection&>(*node);
      for (auto const& [angle, pAgent] : intersection.agents()) {
        if (pAgent->streetId().has_value() && pAgent->streetId().value() == streetId) {
          meanSpeed += pAgent->speed();
          ++n;
        }
      }
    } else if (node->isRoundabout()) {
      auto& roundabout = dynamic_cast<Roundabout&>(*node);
      for (auto const& pAgent : roundabout.agents()) {
        if (pAgent->streetId().has_value() && pAgent->streetId().value() == streetId) {
          meanSpeed += pAgent->speed();
          ++n;
        }
      }
    }
    return meanSpeed / n;
  }

  Measurement<double> FirstOrderDynamics::streetMeanSpeed() const {
    if (this->agents().empty()) {
      return Measurement(0., 0.);
    }
    std::vector<double> speeds;
    speeds.reserve(this->graph().edges().size());
    for (const auto& [streetId, street] : this->graph().edges()) {
      speeds.push_back(this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(speeds);
  }
  Measurement<double> FirstOrderDynamics::streetMeanSpeed(double threshold,
                                                          bool above) const {
    std::vector<double> speeds;
    speeds.reserve(this->graph().edges().size());
    for (const auto& [streetId, street] : this->graph().edges()) {
      if (above) {
        if (street->density(true) > threshold) {
          speeds.push_back(this->streetMeanSpeed(streetId));
        }
      } else {
        if (street->density(true) < threshold) {
          speeds.push_back(this->streetMeanSpeed(streetId));
        }
      }
    }
    return Measurement<double>(speeds);
  }
}  // namespace dsf