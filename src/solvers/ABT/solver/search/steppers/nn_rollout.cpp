/** @file nn_rollout.cpp
 *
 * Contains the implementation for an approximate-nearest-neighbor-based rollout strategy.
 */
#include "nn_rollout.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/Solver.hpp"

#include "solvers/ABT/solver/mappings/actions/ActionMapping.hpp"

namespace abt {

NnRolloutFactory::NnRolloutFactory(Solver *solver, long maxNnComparisons, FloatType maxNnDistance) :
            solver_(solver),
            maxNnComparisons_(maxNnComparisons),
            maxNnDistance_(maxNnDistance),
            nnMap_() {
}

BeliefNode* NnRolloutFactory::findNeighbor(BeliefNode *belief) {
    // A maximum distance of 0 means this function is disabled.
    if (maxNnDistance_ < 0) {
        return nullptr;
    }

    // Initially there is no minimum distance, unless we've already stored a neighbor.
    FloatType minDist = std::numeric_limits<FloatType>::infinity();
    BeliefNode *nearestBelief = nnMap_[belief].neighbor;
    if (nearestBelief != nullptr) {
        minDist = belief->distL1Independent(nearestBelief);
    }

    long numTried = 0;
    for (BeliefNode *otherBelief : solver_->getPolicy()->getNodes()) {
        // Obviously we don't want the belief itself.
        if (belief == otherBelief) {
            continue;
        }

        if (numTried >= maxNnComparisons_) {
            // Stop if we reach the maximum # of comparisons.
            break;
        } else {
            FloatType distance = belief->distL1Independent(otherBelief);
            if (distance < minDist) {
                minDist = distance;
                nearestBelief = otherBelief;
            }
            numTried++;
        }
    }

    // If it's not near enough, we've failed.
    if (minDist > maxNnDistance_) {
        return nullptr;
    }

    // Otherwise update the mapping with the new neighbor.
    nnMap_[belief].neighbor = nearestBelief;
    return nearestBelief;
}

std::unique_ptr<StepGenerator> NnRolloutFactory::createGenerator(SearchStatus &status,
        HistoryEntry const *entry, State const */*state*/, HistoricalData const */*data*/) {
    // Find a neighbor, and use it to make a new generator.
    BeliefNode *neighbor = findNeighbor(entry->getAssociatedBeliefNode());
    return std::make_unique<NnRolloutGenerator>(status, solver_, neighbor);
}

NnRolloutGenerator::NnRolloutGenerator(SearchStatus &status, Solver *solver, BeliefNode *neighbor) :
            StepGenerator(status),
            model_(solver->getModel()),
            currentNeighborNode_(neighbor) {
    // Set the initial status appropriately.
    if (currentNeighborNode_ == nullptr) {
        status_ = SearchStatus::UNINITIALIZED;
    } else{
        status_ = SearchStatus::INITIAL;
    }
}

Model::StepResult NnRolloutGenerator::getStep(HistoryEntry const */*entry*/, State const *state,
        HistoricalData const */*data*/, Action *action) {
    if (currentNeighborNode_ == nullptr) {
        // If we have no neighbor, the NN rollout is finished.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    std::unique_ptr<Action> nextAction = nullptr;
    if (!action) {
        nextAction = currentNeighborNode_->getRecommendedAction();
        action = nextAction.get();
    }

    // Generate a step using the recommended action from the neighboring node.    
    Model::StepResult result = model_->generateStep(*state, *action);

    // getChild() will return nullptr if the child doesn't yet exist => this will be the last step.
    currentNeighborNode_ = currentNeighborNode_->getChild(*action, *result.observation);
    return std::move(result);
}
} /* namespace abt */
