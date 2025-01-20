#pragma once

#include "Edge.hpp"

#include <vector>

namespace dsm {
  class StochasticStreet : public Edge {
  private:
    std::vector<Id> m_movingAgents;
    std::vector<Id> m_queue;
    double m_length;
    double m_maxSpeed;
    int m_nLanes;
    std::string m_name;
    static double m_meanVehicleLength;
  };
}  // namespace dsm