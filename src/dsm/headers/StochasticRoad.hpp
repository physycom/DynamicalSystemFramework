#pragma once

#include "Road.hpp"

namespace dsm {
  class StochasticRoad : public Road {
  protected:
    std::vector<Id> m_movingAgents;
    std::vector<Id> m_exitQueue;
    double m_greenPercentage = 1.1;

  public:
    using Road::Road;

    void setGreenPercentage(double greenPercentage);

    std::vector<Id> const& movingAgents() const;
    std::vector<Id> const& exitQueue() const;
    double greenPercentage() const;

    void enqueue(Id agentId);
    std::optional<Id> dequeue();

    void addAgent(Id agentId) override;

    int nAgents() const override;
    int nMovingAgents() const override;
    int nExitingAgents() const override;
  };
}  // namespace dsm