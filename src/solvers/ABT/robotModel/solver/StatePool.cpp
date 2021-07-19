#include "StatePool.hpp"
#include <iostream>
#include "solvers/ABT/robotModel/RobotState.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/robotModel/solver/StateInfo.hpp"

using std::cout;
using std::endl;

namespace abt
{
OPPTStatePool::OPPTStatePool(std::unique_ptr< StateIndex > stateIndex):
    StatePool(std::move(stateIndex))
{

}

OPPTStatePool::~OPPTStatePool()
{

}

void OPPTStatePool::cleanup()
{
    auto it = stateInfosMap_.begin();
    long cleanedStates = 0;
    while (it != stateInfosMap_.end()) {
        if (static_cast<oppt::StateInfo const*>(it->second.get())->containedInBeliefs_ == 0) {            
            it = stateInfosMap_.erase(it);
            cleanedStates++;
        } else {
            it++;
        }
    }
}

StateInfo* OPPTStatePool::createOrGetInfo(std::unique_ptr<State> state)
{
    StateInfo* info = getInfo(state.get());
    if (info)
        return info;
    auto stateInfo = std::make_unique<oppt::StateInfo>(std::move(state));
    stateInfo->id_ = oppt::UID::getUniqueId();
    auto ret = stateInfosMap_.emplace(stateInfo->getState(), std::move(stateInfo));
    if (ret.second && stateIndex_)
        stateIndex_->addStateInfo(ret.first->second.get());
    return ret.first->second.get();
}

StateInfo* OPPTStatePool::getInfo(State const* state) const
{
    StateInfosMap::const_iterator it = stateInfosMap_.find(state);
    if (it == stateInfosMap_.end())
        return nullptr;
    return it->second.get();

}

StateInfo* OPPTStatePool::getInfoById(long int id) const
{
    for (auto& stateInfoEntry : stateInfosMap_) {
        if (stateInfoEntry.second->id_ == id)
            return stateInfoEntry.second.get();
    }

    return nullptr;
}


long int OPPTStatePool::getNumberOfStates() const
{
    return stateInfosMap_.size();
}

std::vector<State const*> OPPTStatePool::getStates() const
{
    std::vector<State const*> states;
    for (auto& entry : stateInfosMap_) {
        states.push_back(entry.first);
    }

    return states;
}


}
