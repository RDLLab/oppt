#include "StateInfo.hpp"

namespace oppt
{
StateInfo::StateInfo():
    StateInfo(nullptr)
{

}

StateInfo::StateInfo(std::unique_ptr<abt::State> state):
    abt::StateInfo(std::move(state)),
    containedInBeliefs_(0)
{
    
}

StateInfo::~StateInfo()
{

}

long StateInfo::containedInBeliefs() const {
    return containedInBeliefs_;
}

}
