/** @file StatePool.cpp
 *
 * Contains the implementation of the  StatePool class.
 */
#include "StatePool.hpp"

#include <map>                          // for multimap, __alloc_traits<>::value_type
#include <set>                          // for set
#include <unordered_map>                // for unordered_map
#include <unordered_set>                // for unordered_set
#include <utility>                      // for move, pair
#include <iostream>

#include "oppt/global.hpp"                     // for make_unique

#include "StateInfo.hpp"                // for StateInfo, StateInfo::currId

#include "abstract-problem/State.hpp"
#include "changes/ChangeFlags.hpp"               // for ChangeFlags


using std::cout;
using std::endl;

namespace abt
{

StatePool::StatePool(std::unique_ptr<StateIndex> stateIndex) :
    stateInfoMap_(),
    statesByIndex_(),
    stateIndex_(std::move(stateIndex)),
    changedStates_()
{
}

StatePool::~StatePool()
{

}

/* ------------------ Simple getters ------------------- */
StateInfo* StatePool::getInfo(State const* state) const
{
    StateInfoMap::const_iterator it = stateInfoMap_.find(state);
    if (it == stateInfoMap_.end()) {
        return nullptr;
    }
    return it->second;
}
StateInfo* StatePool::getInfoById(long id) const
{
    return statesByIndex_[id].get();
}
StateIndex* StatePool::getStateIndex() const
{
    return stateIndex_.get();
}
long StatePool::getNumberOfStates() const
{
    return statesByIndex_.size();
}

/* ------------------ State lookup ------------------- */
StateInfo* StatePool::createOrGetInfo(std::unique_ptr<State> state)
{
    StateInfo* info = getInfo(state.get());
    if (info != nullptr) {
        return info;
    }

    return add(std::make_unique<StateInfo>(std::move(state)));
}

/* ------------------ Flagging of changes at states ------------------- */
void StatePool::resetChangeFlags(StateInfo* stateInfo)
{
    stateInfo->resetChangeFlags();
    changedStates_.erase(stateInfo);
}
void StatePool::setChangeFlags(StateInfo* stateInfo, ChangeFlags flags)
{
    if (flags != ChangeFlags::UNCHANGED) {
        stateInfo->setChangeFlags(flags);
        changedStates_.insert(stateInfo);
    }
}
void StatePool::resetAffectedStates()
{
    for (StateInfo * stateInfo : changedStates_) {
        stateInfo->resetChangeFlags();
    }
    changedStates_.clear();
}
std::unordered_set<StateInfo*> StatePool::getAffectedStates() const
{
    return changedStates_;
}

std::vector<State const*> StatePool::getStates() const
{
    return std::vector<State const*>();
}

/* ============================ PRIVATE ============================ */


/* ------------------ Mutators for the pool ------------------- */
StateInfo* StatePool::add(std::unique_ptr<StateInfo> newInfo)
{
    try {
        std::pair<StateInfoMap::iterator, bool> ret = (
                    stateInfoMap_.emplace(newInfo->getState(), newInfo.get()));
        StateInfo* stateInfo = ret.first->second;
        if (ret.second) {
            long newId = long(statesByIndex_.size());
            // New state - add to the index.
            long oldId = stateInfo->getId();
            if (oldId != -1 && oldId != newId) {
                std::ostringstream message;
                message << "ERROR: ID mismatch - file says " << oldId;
                message << " but and ID of " << newId << " was assigned.";
                debug::show_message(message.str());
            }
            stateInfo->id_ = newId;
            statesByIndex_.push_back(std::move(newInfo));
            stateIndex_->addStateInfo(stateInfo);
        } else {
            debug::show_message("ERROR: StateInfo already added!!");
        }
        return stateInfo;
    } catch (...) {
        throw;
    }
}
} /* namespace abt */
