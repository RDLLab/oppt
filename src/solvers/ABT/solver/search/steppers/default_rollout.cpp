/** @file default_rollout.cpp
 *
 * Contains the implementation for a basic rollout strategy, which queries the model for a
 * history-based rollout action at every time step.
 */
#include "default_rollout.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/Solver.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"

namespace abt {
/* ------------------------- DefaultRolloutGenerator ------------------------- */
DefaultRolloutGenerator::DefaultRolloutGenerator(SearchStatus &status,
        Solver *solver, long maxNSteps) :
            StepGenerator(status),
            model_(solver->getModel()),
            maxNSteps_(maxNSteps),
            currentNSteps_(0) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult DefaultRolloutGenerator::getStep(HistoryEntry const *entry, State const *state,
        HistoricalData const *data, Action *action) {
    // If we've hit the step limit, we don't generate any more steps.
    if (currentNSteps_ >= maxNSteps_) {
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    // Otherwise, we generate a new step and return it.
    currentNSteps_++;    
    std::unique_ptr<Action> nextAction = nullptr;
    if (!action) {
        nextAction = model_->getRolloutAction(entry, state, data);
        action = nextAction.get();
    }
    return model_->generateStep(*state, *action);
}

/* ------------------------- DefaultRolloutFactory ------------------------- */
DefaultRolloutFactory::DefaultRolloutFactory(Solver *solver, long maxNSteps) :
            solver_(solver),
            maxNSteps_(maxNSteps) {
}

std::unique_ptr<StepGenerator> DefaultRolloutFactory::createGenerator(SearchStatus &status,
        HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<DefaultRolloutGenerator>(status, solver_, maxNSteps_);
}

} /* namespace abt */
