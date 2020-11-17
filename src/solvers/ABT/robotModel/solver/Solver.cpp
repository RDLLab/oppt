#include "Solver.hpp"
#include "solvers/ABT/robotModel/belief/BeliefTree.hpp"
#include "solvers/ABT/solver/StatePool.hpp"
#include "solvers/ABT/solver/Histories.hpp"
#include "solvers/ABT/solver/changes/HistoryCorrector.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

#include "solvers/ABT/solver/belief-estimators/estimators.hpp"
#include "solvers/ABT/solver/mappings/actions/ActionPool.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"
#include "solvers/ABT/solver/mappings/observations/ObservationPool.hpp"
#include "StatePool.hpp"
#include "solvers/ABT/ABTOptions.hpp"
#include "Histories.hpp"
#include "HistorySequence.hpp"
#include <boost/timer.hpp>

namespace oppt
{
ABTSolver::ABTSolver(std::unique_ptr< abt::Model > model):
    abt::Solver(std::move(model))
{

}

void ABTSolver::initializeEmpty()
{
    // Basic initialization.    
    initialize();    

    // Create new instances of these.
    actionPool_ = model_->createActionPool(this);
    observationPool_ = model_->createObservationPool(this);    

    // Initialize the root node properly.
    static_cast<oppt::BeliefTree*>(policy_.get())->initializeRoot();    
}


void ABTSolver::initialize()
{
    // Core data structures    
    if (!hasChanges_ && !interactive_) {
        statePool_ = std::make_unique<abt::OPPTStatePool>(nullptr);
    } else {
        statePool_ = std::make_unique<abt::OPPTStatePool>(model_->createStateIndex());
    }
    histories_ = std::make_unique<abt::OPPTHistories>();
    policy_ = std::make_unique<oppt::BeliefTree>(this);
    static_cast<oppt::BeliefTree*>(policy_.get())->reset();

    // Serializable model-specific customizations
    actionPool_ = nullptr;
    observationPool_ = nullptr;

    // Possible model-specific customizations
    historyCorrector_ = model_->createHistoryCorrector(this);
    searchStrategy_ = model_->createSearchStrategy(this);
    recommendationStrategy_ = model_->createRecommendationSelectionStrategy(this);
    estimationStrategy_ = model_->createEstimationStrategy(this);

}

long ABTSolver::cleanHistories(abt::BeliefNode* currNode)
{
    long cleanedHistories = 0;
    long toDelete = histories_->sequencesById_.size();
    for (auto & historyEntry : currNode->particles_) {
        static_cast<abt::OPPTHistorySequence*>(historyEntry->owningSequence_)->markToDelete_ = false;
        toDelete--;
    }

    std::vector<std::unique_ptr<abt::HistorySequence>> sequencesToDelete(toDelete);
    std::vector<std::unique_ptr<abt::HistorySequence>> remainingSequences(histories_->sequencesById_.size() - toDelete);
    long counter = 0;
    long counter2 = 0;
    for (size_t i = 0; i != histories_->sequencesById_.size(); ++i) {
        if (static_cast<abt::OPPTHistorySequence*>(histories_->sequencesById_[i].get())->markToDelete_) {
            sequencesToDelete[counter] = std::move(histories_->sequencesById_[i]);
            counter++;
        } else {
            remainingSequences[counter2] = std::move(histories_->sequencesById_[i]);
            remainingSequences[counter2]->id_ = counter2;
            counter2++;
        }
    }

    for (auto & sequence : sequencesToDelete) {
        sequence->erase();
    }

    histories_->sequencesById_ = std::move(remainingSequences);
    for (size_t i = 0; i != histories_->sequencesById_.size(); ++i) {
        histories_->sequencesById_[i]->id_ = i;
        static_cast<abt::OPPTHistorySequence*>(histories_->sequencesById_[i].get())->markToDelete_ = true;
    }

    return cleanedHistories;
}

void ABTSolver::singleSearch(abt::BeliefNode* startNode, abt::StateInfo* startStateInfo, long maximumDepth, abt::Action *action)
{
    abt::HistorySequence* sequence = histories_->createSequence();
    abt::HistoryEntry* firstEntry = sequence->addEntry();
    firstEntry->registerState(startStateInfo);
    if (startNode == getPolicy()->getRoot()) {
        firstEntry->registerNode(startNode);
    } else {
        firstEntry->registerNode(startNode, false);
    }    
    
    continueSearch(sequence, maximumDepth, action);    
}



}
