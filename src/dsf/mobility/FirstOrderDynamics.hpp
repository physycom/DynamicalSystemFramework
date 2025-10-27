#pragma once

#include "RoadDynamics.hpp"

namespace dsf::mobility {
  class FirstOrderDynamics : public RoadDynamics<Delay> {
    double m_alpha;
    double m_speedFluctuationSTD;

    double m_speedFactor(double const& density) const;

    double m_streetEstimatedTravelTime(
        std::unique_ptr<Street> const& pStreet) const override;

  public:
    /// @brief Construct a new First Order Dynamics object
    /// @param graph The graph representing the network
    /// @param useCache If true, the cache is used (default is false)
    /// @param seed The seed for the random number generator (default is std::nullopt)
    /// @param alpha The minimum speed rate (default is 0)
    /// @param weightFunction The dsf::PathWeight function to use for the pathfinding (default is dsf::PathWeight::TRAVELTIME)
    /// @param weightTreshold The weight threshold for the pathfinding (default is std::nullopt)
    FirstOrderDynamics(RoadNetwork& graph,
                       bool useCache = false,
                       std::optional<unsigned int> seed = std::nullopt,
                       double alpha = 0.,
                       PathWeight const weightFunction = PathWeight::TRAVELTIME,
                       std::optional<double> weightTreshold = std::nullopt);
    /// @brief Set the speed of an agent
    /// @param agentId The id of the agent
    /// @throw std::invalid_argument, If the agent is not found
    void setAgentSpeed(std::unique_ptr<Agent> const& pAgent) override;
    /// @brief Set the standard deviation of the speed fluctuation
    /// @param speedFluctuationSTD The standard deviation of the speed fluctuation
    /// @throw std::invalid_argument, If the standard deviation is negative
    void setSpeedFluctuationSTD(double speedFluctuationSTD);
    /// @brief Get the mean speed of a street in \f$m/s\f$
    /// @return double The mean speed of the street or street->maxSpeed() if the street is empty
    /// @details The mean speed of a street is given by the formula:
    /// \f$ v_{\text{mean}} = v_{\text{max}} \left(1 - \frac{\alpha}{2} \left( n - 1\right)  \right) \f$
    /// where \f$ v_{\text{max}} \f$ is the maximum speed of the street, \f$ \alpha \f$ is the minimum speed rateo divided by the capacity
    /// and \f$ n \f$ is the number of agents in the street
    double streetMeanSpeed(Id streetId) const override;
    /// @brief Get the mean speed of the streets in \f$m/s\f$
    /// @return Measurement The mean speed of the agents and the standard deviation
    Measurement<double> streetMeanSpeed() const override;
    /// @brief Get the mean speed of the streets with density above or below a threshold in \f$m/s\f$
    /// @param threshold The density threshold to consider
    /// @param above If true, the function returns the mean speed of the streets with a density above the threshold, otherwise below
    /// @return Measurement The mean speed of the agents and the standard deviation
    Measurement<double> streetMeanSpeed(double threshold, bool above) const override;
  };
}  // namespace dsf::mobility