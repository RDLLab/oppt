#ifndef __ABT_SOLVER_SOLVER_HPP__
#define __ABT_SOLVER_SOLVER_HPP__
#include "oppt/opptCore/core.hpp"
#include "solvers/ABT/solver/Solver.hpp"

namespace solvers
{
class ABT;
class ABTMultithreaded;
}

namespace oppt
{
class ABTSolver: public abt::Solver
{
public:
    friend class solvers::ABT;
    friend class solvers::ABTMultithreaded;
    ABTSolver(std::unique_ptr<abt::Model> model);
    virtual ~ABTSolver() = default;
    _NO_COPY_OR_MOVE(ABTSolver);

    virtual void initializeEmpty() override;

    long cleanHistories(abt::BeliefNode* currNode);

protected:
    virtual void initialize() override;

    virtual void singleSearch(abt::BeliefNode* startNode, abt::StateInfo* startStateInfo, long maximumDepth, abt::Action *action = nullptr) override;

    bool hasChanges_ = false;

    bool interactive_ = false;
};


}

#endif
