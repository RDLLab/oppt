/** @file DefaultHistoryCorrector.cpp
 *
 * Contains the implementation of the DefaultHistoryCorrector class.
 */
#include "DefaultHistoryCorrector.hpp"

#include <memory>

#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/solver/HistorySequence.hpp"
#include "solvers/ABT/solver/Solver.hpp"
#include "solvers/ABT/solver/StatePool.hpp"

#include "solvers/ABT/solver/abstract-problem/Model.hpp"
#include "solvers/ABT/solver/abstract-problem/State.hpp"

#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"

namespace abt {
DefaultHistoryCorrector::DefaultHistoryCorrector(Solver *solver, HeuristicFunction heuristic) :
            HistoryCorrector(solver),
            heuristic_(heuristic) {
}

bool DefaultHistoryCorrector::reviseSequence(HistorySequence *sequence) {
    if (sequence->endAffectedIdx_ < sequence->startAffectedIdx_) {
        debug::show_message("WARNING: Sequence to update has no affected entries!?");
        return true;
    }

    // The ID of the entry at which this sequence goes down a different path in the tree to
    // its previous path; -1 if no divergence has occurred.
    long divergingEntryId = -1;

    bool hitIllegalAction = false; // True iff we hit an illegal action.
    bool hitTerminalState = false; // True iff the sequence terminated prematurely.

    std::vector<std::unique_ptr<HistoryEntry>>::iterator historyIterator =
            (sequence->entrySequence_.begin() + sequence->startAffectedIdx_);
    std::vector<std::unique_ptr<HistoryEntry>>::iterator firstUnchanged =
            (sequence->entrySequence_.begin() + sequence->endAffectedIdx_ + 1);

    // Extra variables for use in the iteration.
    HistoryEntry *entry = historyIterator->get(); // The current history entry.
    State const *state = entry->getState(); // The current state.
    // The actual current node that should be associated with this history entry.
    BeliefNode *actualCurrentNode = entry->getAssociatedBeliefNode();

    while (historyIterator != firstUnchanged) {
        // Check for early termination.
        hitTerminalState = getModel()->isTerminal(*state);
        if (hitTerminalState || entry->action_ == nullptr) {
            break;
        }

        // Deleted states should never occur in the update phase.
        if (changes::has_flags(entry->changeFlags_, ChangeFlags::DELETED)) {
            debug::show_message("ERROR: deleted state in updateSequence.");
        }

        HistoryEntry *nextEntry = (historyIterator + 1)->get();
        if (changes::has_flags(entry->changeFlags_, ChangeFlags::TRANSITION)) {

            // Check for illegal actions.
            ActionMappingEntry *mappingEntry = actualCurrentNode->getMapping()->getEntry(
                    *entry->action_);
            if (mappingEntry != nullptr && !mappingEntry->isLegal()) {
                hitIllegalAction = true;
                break;
            }


            entry->transitionParameters_ = getModel()->generateTransition(*state, *entry->action_);
            std::unique_ptr<State> nextState = getModel()->generateNextState(*state,
                    *entry->action_, entry->transitionParameters_.get());            
            StateInfo *nextStateInfo = getSolver()->getStatePool()->createOrGetInfo(std::move(nextState));
            if (nextStateInfo != nextEntry->getStateInfo()) {
                nextEntry->registerState(nextStateInfo);
                if (historyIterator + 1 == firstUnchanged) {
                    firstUnchanged++;
                }
                entry->setChangeFlags(ChangeFlags::OBSERVATION | ChangeFlags::REWARD);
                // Different state, so we must reset the flags.
                nextEntry->changeFlags_ = (ChangeFlags::REWARD | ChangeFlags::TRANSITION |
                        ChangeFlags::HEURISTIC);
                // Since it's a different state we may also need to update the heuristic.
            }
        }

        if (changes::has_flags(entry->changeFlags_, ChangeFlags::REWARD)) {
            FloatType oldReward = entry->immediateReward_;
            entry->immediateReward_ = getModel()->generateReward(*state,
                    *entry->action_, entry->transitionParameters_.get(), nextEntry->getState());

            // If we haven't diverged yet, we update the difference right now.
            if (divergingEntryId == -1 && entry->immediateReward_ != oldReward) {
                getSolver()->updateImmediate(actualCurrentNode,
                        *entry->action_, *entry->observation_,
                        entry->immediateReward_ - oldReward , 0);
            }

        }

        if (changes::has_flags(entry->changeFlags_, ChangeFlags::OBSERVATION)) {
            std::unique_ptr<Observation> newObservation = (getModel()->generateObservation(
                    state, *entry->action_, entry->transitionParameters_.get(),
                    *nextEntry->getState()));

            ObservationMapping *obsMap = actualCurrentNode->getMapping()->getActionNode(
                    *entry->action_)->getMapping();
            if (obsMap->getEntry(*entry->observation_) == obsMap->getEntry(*newObservation)) {
                if (divergingEntryId == -1) {
                    divergingEntryId = entry->entryId_;
                    // We have diverged; this means the rest of the sequence should be negated.
                    getSolver()->updateSequence(sequence, -1, divergingEntryId, false);
                    // Now that we've negated the sequence, we can start updating node pointers.
                }

                entry->observation_ = std::move(newObservation);
            }
        }
        entry->resetChangeFlags(); // Reset the change flags for this entry.


        // Update our iteration variables.
        if (divergingEntryId != -1) {
            // Diverged => create a new node.
            actualCurrentNode = actualCurrentNode->createOrGetChild(*entry->getAction(),
                    *entry->getObservation());
            historyIterator++;
            entry = historyIterator->get();
            entry->registerNode(actualCurrentNode);
            state = entry->getState();
        } else {
            // No divergence => use the previously registered node.
            historyIterator++;
            entry = historyIterator->get();
            actualCurrentNode = entry->getAssociatedBeliefNode();
            state = entry->getState();
        }
    }

    if (entry->action_ == nullptr) {
        // We hit the last entry in the sequence => handle it as a special case.

        if (changes::has_flags(entry->changeFlags_, ChangeFlags::HEURISTIC)) {
            // We only have to deal with changes to the heuristic value if we've flagged it.
            FloatType oldValue = entry->immediateReward_;
            if (hitTerminalState) {
                entry->immediateReward_ = 0;
            } else {
                entry->immediateReward_ = heuristic_(entry, state,
                        actualCurrentNode->getHistoricalData());
            }

            // If no divergence occurred we update the difference in heuristic values here and now.
            if (divergingEntryId == -1) {
                getSolver()->updateEstimate(actualCurrentNode, entry->immediateReward_ - oldValue, 0);
            }
        }
    } else {
        // The changes did not reach the end of the sequence.

        // A terminal state or an illegal action causes early termination.
        if (hitTerminalState || hitIllegalAction) { // Early termination!
            if (divergingEntryId == -1) {
                // No divergence => not yet negated => negate it from here onwards right now.
                getSolver()->updateSequence(sequence, -1, entry->entryId_, false);
            }

            // Now we have to erase all of the remaining entries in the sequence.
            sequence->erase(entry->entryId_ + 1);

            if (entry->entryId_ > 0) {
                // Due to the early termination we must negate a continuation.
                getSolver()->updateEstimate(actualCurrentNode, 0, -1);
            }

            // Now we must set the values for this entry properly.
            entry->action_ = nullptr;
            entry->transitionParameters_ = nullptr;
            entry->observation_ = nullptr;
            entry->immediateReward_ = 0;
        }

    }

    entry->resetChangeFlags(); // Reset the change flags for the last affected entry.

    // If there was divergence, we must update the rest of the sequence.
    if (divergingEntryId != -1) {
        // Update the node pointers for the rest of the sequence.
        while (entry->getAction() != nullptr) {
            actualCurrentNode = actualCurrentNode->createOrGetChild(*entry->getAction(),
                    *entry->getObservation());
            historyIterator++;
            entry = historyIterator->get();
            entry->registerNode(actualCurrentNode);
        }

        // Now we backup the sequence.
        getSolver()->updateSequence(sequence, +1, divergingEntryId, false);
    }

    // Reset change flags for the sequence as a whole.
    sequence->resetChangeFlags();

    // If we hit an illegal action, the search algorithm will need to extend this sequence.
    return !hitIllegalAction;
}

} /* namespace abt */
