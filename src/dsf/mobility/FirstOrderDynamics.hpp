#pragma once

#include "RoadDynamics.hpp"

namespace dsf::mobility {
  class FirstOrderDynamics : public RoadDynamics<Delay> {
    double m_alpha;
    double m_speedFluctuationSTD;

    double m_speedFactor(double const& density) const final;

    double m_streetEstimatedTravelTime(std::unique_ptr<Street> const& pStreet) const final;

    void m_dumpSimInfo() const final;

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
  };
}  // namespace dsf::mobility