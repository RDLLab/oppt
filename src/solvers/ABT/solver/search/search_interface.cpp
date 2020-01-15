/** @file search_interface.cpp
 *
 * Provides some basic implementations of classes for generating history sequences.
 *
 * See BasicSearchStrategy::extendSequence() for the core implementation used to extend and
 * backup most history sequences.
 */
#include "search_interface.hpp"

#include <functional>
#include <memory>

#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/Solver.hpp"
#include "solvers/ABT/solver/StatePool.hpp"

#include "solvers/ABT/solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "SearchStatus.hpp"

#include "action-choosers/choosers.hpp"
#include <signal.h>
#include <boost/timer.hpp>

using std::cout;
using std::endl;

namespace abt
{
/* ----------------------- StepGenerator ------------------------- */
StepGenerator::StepGenerator(SearchStatus& status) :
    status_(status)
{
}

/* ------------------- StagedStepGenerator --------------------- */
StagedStepGenerator::StagedStepGenerator(SearchStatus& status,
        std::vector<std::unique_ptr<StepGeneratorFactory>> const& factories,
        HistoryEntry const* entry, State const* state, HistoricalData const* data) :
    StepGenerator(status),
    factories_(factories),
    iterator_(factories.cbegin()),
    generator_(nullptr)
{
    generator_ = (*iterator_)->createGenerator(status_, entry, state, data);
    iterator_++;
}

Model::StepResult StagedStepGenerator::getStep(HistoryEntry const* entry, State const* state,
        HistoricalData const* data, Action *action)
{
    Model::StepResult result = generator_->getStep(entry, state, data, action);

    // If the action was null, we may need to continue on to the next generator.
    while (result.action == nullptr) {
        // Go on to the next generator only if we're out of steps and there's more left.
        if (status_ != SearchStatus::OUT_OF_STEPS || iterator_ == factories_.cend()) {
            generator_ = nullptr;
            return result;
        }

        generator_ = (*iterator_)->createGenerator(status_, entry, state, data);
        iterator_++;
        result = generator_->getStep(entry, state, data, action);
    }
    return result;
}

/* ------------------- StagedStepGeneratorFactory --------------------- */
StagedStepGeneratorFactory::StagedStepGeneratorFactory(
    std::vector<std::unique_ptr<StepGeneratorFactory>> factories) :
    factories_(std::move(factories))
{
}

std::unique_ptr<StepGenerator> StagedStepGeneratorFactory::createGenerator(SearchStatus& status,
        HistoryEntry const* entry, State const* state, HistoricalData const* data)
{
    return std::make_unique<StagedStepGenerator>(status, factories_, entry, state, data);
}

/* ------------------- BasicSearchStrategy --------------------- */
BasicSearchStrategy::BasicSearchStrategy(Solver* solver,
        std::unique_ptr<StepGeneratorFactory> factory, HeuristicFunction heuristic) :
    solver_(solver),
    factory_(std::move(factory)),
    heuristic_(heuristic)
{
}

/** This is the default implementation for extending and backing up a history sequence. */
SearchStatus BasicSearchStrategy::extendAndBackup(HistorySequence* sequence, long maximumDepth, Action *const action)
{
    // We extend from the last entry in the sequence.
    HistoryEntry* firstEntry = sequence->getLastEntry();
    long firstEntryId = firstEntry->getId();

    HistoryEntry* currentEntry = firstEntry;
    BeliefNode* currentNode = currentEntry->getAssociatedBeliefNode();

    SearchStatus status = SearchStatus::UNINITIALIZED;
    std::unique_ptr<StepGenerator> generator = factory_->createGenerator(status, currentEntry,
            currentEntry->getState(), currentNode->getHistoricalData());
    if (status == SearchStatus::UNINITIALIZED) {
        // Failure to initialize => return this status.
        return status;
    }

    // We check for some invalid possibilities in order to deal with them more cleanly.
    Model* model = solver_->getModel();

    boost::timer t0;
    if (!currentEntry)
        cout << "Current entry is null??" << endl;
    if (!(currentEntry->getState()))
        cout << "Current state is null??" << endl;
    if (model->isTerminal(*currentEntry->getState())) {
        //debug::show_message("WARNING: Attempted to continue sequence"
        //        " from a terminal state.");
        return SearchStatus::ERROR;
    } else if (currentEntry->getAction() != nullptr) {
        debug::show_message("ERROR: The last in the sequence already has an action!?");
        return SearchStatus::ERROR;
    } else if (currentEntry->immediateReward_ != 0) {
        debug::show_message("ERROR: The last in the sequence has a nonzero reward!?");
        return SearchStatus::ERROR;
    }    

    bool useProvidedAction = false;
    if (action) {
        useProvidedAction = true;        
    }

    while (true) {
        if (currentNode->getDepth() >= maximumDepth) {
            // We've hit the depth limit, so we can't generate any more steps in the sequence.
            status = SearchStatus::OUT_OF_STEPS;
            break;
        }

        // Step the search forward.        
        Model::StepResult result;
        if (useProvidedAction) {
            result = generator->getStep(currentEntry, currentEntry->getState(),
                                        currentNode->getHistoricalData(), action);
            // Make sure that if there's an input action provided, use it only in the first step
            useProvidedAction = false;
        } else {
            result = generator->getStep(currentEntry, currentEntry->getState(),
                                        currentNode->getHistoricalData(), nullptr);
        }

        // Null action => stop the search.
        if (result.action == nullptr) {
            break;
        }                

        // Set the parameters of the current history entry using the ones we got from the result.
        currentEntry->immediateReward_ = result.reward;
        currentEntry->action_ = std::move(result.action);
        currentEntry->transitionParameters_ = std::move(result.transitionParameters);
        currentEntry->observation_ = std::move(result.observation);

        // Create the child belief node, and set the current node to be that node.
        BeliefNode* nextNode = currentNode->createOrGetChild(*currentEntry->action_,
                               *currentEntry->observation_);
        currentNode = nextNode;

        // Now we create a new history entry and step the history forward.
        StateInfo* nextStateInfo = solver_->getStatePool()->createOrGetInfo(std::move(result.nextState));
        result.nextState = nullptr;
        currentEntry = sequence->addEntry();

        // Register the new history entry with its state, and with its associated belief node.
        currentEntry->registerState(nextStateInfo);
        currentEntry->registerNode(currentNode);

        if (result.isTerminal) {            
            nextStateInfo->setTerminal(true);
            // Terminal state => search complete.
            status = SearchStatus::FINISHED;
            break;
        }
    }

    // OUT_OF_STEPS => must calculated a heuristic estimate.
    if (status == SearchStatus::OUT_OF_STEPS) {        
        currentEntry->immediateReward_ = heuristic_(currentEntry, currentEntry->getState(),
                                         currentNode->getHistoricalData());
        status = SearchStatus::FINISHED;
    }

    if (status == SearchStatus::FINISHED) {
        // Now that we're finished, we back up the sequence.
        if (firstEntryId > 0 && currentEntry->getId() > firstEntryId) {
            // If we've extended a previously terminating sequence, we have to add a continuation.
            solver_->updateEstimate(firstEntry->getAssociatedBeliefNode(), 0, +1);
        }
        // We only do a partial backup along the newly generated part of the sequence.
        solver_->updateSequence(sequence, +1, firstEntryId);
    } else {
        // This shouldn't happen => print out an error message.
        if (status == SearchStatus::UNINITIALIZED) {
            debug::show_message("ERROR: Search algorithm could not initialize.");
        } else if (status == SearchStatus::INITIAL) {
            debug::show_message("ERROR: Search algorithm initialized but did not run!?");
        } else if (status == SearchStatus::ERROR) {
            debug::show_message("ERROR: Error in search algorithm!");
        } else {
            debug::show_message("ERROR: Invalid search status.");
        }
    }

    // Finally, we just return the status.
    return status;
}

std::unique_ptr<Action> MaxRecommendedActionStrategy::getAction(const BeliefNode* belief)
{
    return choosers::max_action(belief);
}

GpsMaxRecommendedActionStrategy::GpsMaxRecommendedActionStrategy(const choosers::GpsMaxRecommendationOptions& theOptions): options(theOptions) {}

std::unique_ptr<Action> GpsMaxRecommendedActionStrategy::getAction(const BeliefNode* belief)
{
    return choosers::gps_max_action(belief, options).action;
}


} /* namespace abt */
