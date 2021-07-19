/** @file ucb_search.cpp
 *
 * Contains the basic implementation code for a UCB-based search strategy.
 */
#include "ucb_search.hpp"

#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/Solver.hpp"

#include "solvers/ABT/solver/search/action-choosers/choosers.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"

namespace abt {
UcbStepGenerator::UcbStepGenerator(SearchStatus &status, Solver *solver,
                                   FloatType explorationCoefficient) :
    StepGenerator(status),
    model_(solver->getModel()),
    explorationCoefficient_(explorationCoefficient),
    choseUnvisitedAction_(false) {
    status_ = SearchStatus::INITIAL;
}

Model::StepResult UcbStepGenerator::getStep(HistoryEntry const *entry, State const *state,
        HistoricalData const */*data*/, Action *action) {
    // If we previously chose a new action that hadn't been tried before, UCB is over.
    if (choseUnvisitedAction_) {
        // We've reached the new leaf node - this search is over.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    // Retrieve the mapping.
    BeliefNode *currentNode = entry->getAssociatedBeliefNode();
    ActionMapping *mapping = currentNode->getMapping();
    std::unique_ptr<Action> nextAction = nullptr;
    if (!action) {        
        nextAction = mapping->getNextActionToTry();             
        if (nextAction != nullptr) {
            // If there are unvisited actions, we take one, and we're finished with UCB search.
            choseUnvisitedAction_ = true;
        } else {
            // Use UCB to get the best action.
            nextAction = choosers::ucb_action(currentNode, explorationCoefficient_);            
        }

        // NO action -> error!
        if (nextAction == nullptr) {            
            debug::show_message("ERROR: node has no actions!?");
            status_ = SearchStatus::ERROR;
            return Model::StepResult { };
        }        
    } else {        
        nextAction = action->copy();
    }

    // Use the model to generate the step.
    return model_->generateStep(*state, *nextAction);
}

UcbStepGeneratorFactory::UcbStepGeneratorFactory(Solver *solver, FloatType explorationCoefficient) :
    solver_(solver),
    explorationCoefficient_(explorationCoefficient) {
}

std::unique_ptr<StepGenerator> UcbStepGeneratorFactory::createGenerator(SearchStatus &status,
        HistoryEntry const */*entry*/, State const */*state*/, HistoricalData const */*data*/) {
    return std::make_unique<UcbStepGenerator>(status, solver_, explorationCoefficient_);
}
} /* namespace abt */
