#ifndef __OPPT_HISTORY_SEQUENCE_
#define __OPPT_HISTORY_SEQUENCE_
#include "solvers/ABT/solver/HistorySequence.hpp"

namespace oppt
{
class ABTSolver;
}

namespace abt
{
class OPPTHistorySequence: public HistorySequence
{
public:
    friend class oppt::ABTSolver;
    /** Constructs an empty history sequence, with no ID assigned. */
    OPPTHistorySequence();
    /** Constructs an empty history sequence, assigning the given ID. */
    OPPTHistorySequence(long id);
    
protected:
    virtual abt::HistoryEntry* addEntry();

protected:
    bool markToDelete_ = true;

};
}

#endif
