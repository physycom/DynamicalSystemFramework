#include "FirstOrderDynamics.hpp"

namespace dsm {
  FirstOrderDynamics::FirstOrderDynamics(Graph& graph,
                                         std::optional<unsigned int> seed,
                                         double alpha)
      : RoadDynamics<Delay>(graph, seed), m_alpha{0.}, m_speedFluctuationSTD{0.} {
    if (alpha < 0. || alpha > 1.) {
      throw std::invalid_argument(buildLog(std::format(
          "The minimum speed rateo must be between 0 and 1, but it is {}", alpha)));
    } else {
      m_alpha = alpha;
    }
    double globMaxTimePenalty{0.};
    for (const auto& [streetId, street] : this->m_graph.streetSet()) {
      globMaxTimePenalty =
          std::max(globMaxTimePenalty,
                   std::ceil(street->length() / ((1. - m_alpha) * street->maxSpeed())));
    }
    if (globMaxTimePenalty > static_cast<double>(std::numeric_limits<Delay>::max())) {
      throw std::overflow_error(
          buildLog(std::format("The maximum time penalty ({}) is greater than the "
                               "maximum value of delay_t ({})",
                               globMaxTimePenalty,
                               std::numeric_limits<Delay>::max())));
    }
  }

  void FirstOrderDynamics::setAgentSpeed(Size agentId) {
    const auto& agent{this->m_agents[agentId]};
    const auto& street{this->m_graph.streetSet()[agent->streetId().value()]};
    double speed{street->maxSpeed() * (1. - m_alpha * street->density(true))};
    if (m_speedFluctuationSTD > 0.) {
      std::normal_distribution<double> speedDist{speed, speed * m_speedFluctuationSTD};
      speed = speedDist(this->m_generator);
    }
    speed < 0. ? agent->setSpeed(street->maxSpeed() * (1. - m_alpha))
               : agent->setSpeed(speed);
  }

  void FirstOrderDynamics::setSpeedFluctuationSTD(double speedFluctuationSTD) {
    if (speedFluctuationSTD < 0.) {
      throw std::invalid_argument(
          buildLog("The speed fluctuation standard deviation must be positive."));
    }
    m_speedFluctuationSTD = speedFluctuationSTD;
  }

  double FirstOrderDynamics::streetMeanSpeed(Id streetId) const {
    const auto& street{this->m_graph.streetSet().at(streetId)};
    if (street->nAgents() == 0) {
      return street->maxSpeed();
    }
    double meanSpeed{0.};
    Size n{0};
    if (street->nExitingAgents() == 0) {
      n = static_cast<Size>(street->waitingAgents().size());
      double alpha{m_alpha / street->capacity()};
      meanSpeed = street->maxSpeed() * n * (1. - 0.5 * alpha * (n - 1.));
    } else {
      for (const auto& agentId : street->waitingAgents()) {
        meanSpeed += this->m_agents.at(agentId)->speed();
        ++n;
      }
      for (auto const& queue : street->exitQueues()) {
        for (const auto& agentId : queue) {
          meanSpeed += this->m_agents.at(agentId)->speed();
          ++n;
        }
      }
    }
    const auto& node = this->m_graph.nodeSet().at(street->nodePair().second);
    if (node->isIntersection()) {
      auto& intersection = dynamic_cast<Intersection&>(*node);
      for (const auto& [angle, agentId] : intersection.agents()) {
        const auto& agent{this->m_agents.at(agentId)};
        if (agent->streetId().has_value() && agent->streetId().value() == streetId) {
          meanSpeed += agent->speed();
          ++n;
        }
      }
    } else if (node->isRoundabout()) {
      auto& roundabout = dynamic_cast<Roundabout&>(*node);
      for (const auto& agentId : roundabout.agents()) {
        const auto& agent{this->m_agents.at(agentId)};
        if (agent->streetId().has_value() && agent->streetId().value() == streetId) {
          meanSpeed += agent->speed();
          ++n;
        }
      }
    }
    return meanSpeed / n;
  }

  Measurement<double> FirstOrderDynamics::streetMeanSpeed() const {
    if (this->m_agents.size() == 0) {
      return Measurement(0., 0.);
    }
    std::vector<double> speeds;
    speeds.reserve(this->m_graph.streetSet().size());
    for (const auto& [streetId, street] : this->m_graph.streetSet()) {
      speeds.push_back(this->streetMeanSpeed(streetId));
    }
    return Measurement<double>(speeds);
  }
  Measurement<double> FirstOrderDynamics::streetMeanSpeed(double threshold,
                                                          bool above) const {
    if (this->m_agents.size() == 0) {
      return Measurement(0., 0.);
    }
    std::vector<double> speeds;
    speeds.reserve(this->m_graph.streetSet().size());
    for (const auto& [streetId, street] : this->m_graph.streetSet()) {
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
}  // namespace dsm