#ifndef _OPPT_HISTORIES_HPP_
#define _OPPT_HISTORIES_HPP_
#include "solvers/ABT/solver/Histories.hpp"

namespace abt
{
class OPPTHistories: public Histories
{
public:
    OPPTHistories();
protected:    
    virtual HistorySequence* createSequence() override;
};
}

#endif
