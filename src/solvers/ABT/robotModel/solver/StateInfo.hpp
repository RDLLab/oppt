#ifndef _OPPT_STATE_INFO_HPP_
#define _OPPT_STATE_INFO_HPP_
#include "solvers/ABT/solver/StateInfo.hpp"

namespace abt
{
class OPPTStatePool;
class Solver;
}

namespace oppt
{
class BeliefNode;

class StateInfo: public abt::StateInfo
{
public:
    friend class BeliefNode;
    friend class abt::OPPTStatePool;
    friend class abt::Solver;

    StateInfo();

    StateInfo(std::unique_ptr<abt::State> state);

    virtual ~StateInfo();
    
    long containedInBeliefs() const;

protected:
    long containedInBeliefs_;

};
}

#endif
