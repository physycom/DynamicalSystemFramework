#include "StochasticRoad.hpp"

#include <algorithm>

namespace dsm {
    std::vector<Id> const& StochasticRoad::movingAgents() const {
        return m_movingAgents;
    }
    std::vector<Id> const& StochasticRoad::exitQueue() const {
        return m_exitQueue;
    }

    void StochasticRoad::addAgent(Id agentId) {
        if (std::find(m_movingAgents.cbegin(), m_movingAgents.cend(), agentId) != m_movingAgents.cend()) {
            throw std::invalid_argument(buildLog(std::format("Agent ({}) already in the street.", agentId)));
        }
        m_movingAgents.push_back(agentId);
    }

    void StochasticRoad::enqueue(Id agentId) {
        if (std::find(m_movingAgents.cbegin(), m_movingAgents.cend(), agentId) == m_movingAgents.cend()) {
            throw std::invalid_argument(buildLog(std::format("Agent ({}) not in the street.", agentId)));
        }
        if (std::find(m_exitQueue.cbegin(), m_exitQueue.cend(), agentId) != m_exitQueue.cend()) {
            throw std::invalid_argument(buildLog(std::format("Agent ({}) already in the queue.", agentId)));
        }
        m_movingAgents.erase(std::remove(m_movingAgents.begin(), m_movingAgents.end(), agentId), m_movingAgents.end());
        m_exitQueue.push_back(agentId);
    }
    std::vector<Id> dequeue() {
        std::vector<Id> agents;
        agents.reserve(this->m_transportCappacity);
        for 
    }
}