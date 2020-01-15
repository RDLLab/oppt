#ifndef __ABT_BELIEF_TREE_HPP__
#define __ABT_BELIEF_TREE_HPP__
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "BeliefNode.hpp"
#include "solvers/ABT/robotModel/solver/Solver.hpp"

namespace oppt
{
class BeliefTree: public abt::BeliefTree
{
    friend class oppt::ABTSolver;
public:
    BeliefTree(abt::Solver* solver);
    
protected:
    virtual abt::BeliefNode* reset() override;
    
    /** Adds the given node to the index of nodes. */
    virtual void addNode(BeliefNode *node);

    /** Removes the given node from the index of nodes. */
    virtual void removeNode(BeliefNode *node);
    
    long counter_;
};
}

#endif
