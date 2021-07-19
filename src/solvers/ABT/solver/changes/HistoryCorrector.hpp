/** @file HistoryCorrector.hpp
 *
 * Defines an abstract base class for correcting history sequences.
 */
#ifndef SOLVER_HISTORYCORRECTOR_HPP_
#define SOLVER_HISTORYCORRECTOR_HPP_

#include <unordered_set>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/Solver.hpp"

namespace abt {
class HistorySequence;
class Model;

/** An abstract base class for correcting history sequences that have been affected by changes.
 *
 * The core virtual method is reviseSequence(), which should modify the given history sequence
 * in-place.
 *
 * This class also allows for distinct handling of history sequences in batches - the default
 * reviseHistories() simply calls reviseSequence() on each individual sequence, but it can be
 * overridden for a different approach.
 */
class HistoryCorrector {
public:
    /** Constructs a new HistoryCorrector, which will be associated with the given Solver. */
    HistoryCorrector(Solver *solver) :
        solver_(solver) {
    }
    virtual ~HistoryCorrector() = default;
    _NO_COPY_OR_MOVE(HistoryCorrector);

    /** Revises all of the history sequences in the given set.
     *
     * Any sequences left in the set after this method finishes are considered incomplete, and will
     * be continued via the solver's default search algorithm.
     *
     * By default this method simply calls reviseSequence() on each sequence, but it can be
     * overridden to take a custom approach to dealing with all of the sequences.
     */
    virtual void reviseHistories(
            std::unordered_set<HistorySequence *> &affectedSequences) {
        for (auto it = affectedSequences. begin(); it != affectedSequences.end(); ) {
            HistorySequence *sequence = *it;
            if (reviseSequence(sequence)) {
                // Successful revision => remove it from the set.
                it = affectedSequences.erase(it);
            } else {
                // Search required => leave it in the set.
                it++;
            }
        }
    }

    /** Revises the given sequence, and returns true if the revision was fully successful.
     *
     * Incomplete revisions must later be extended by the search algorithm (e.g. UCB) to
     * avoid having bad sequences in the tree.
     */
    virtual bool reviseSequence(HistorySequence *sequence) = 0;

    /** Returns the solver used with this corrector. */
    virtual Solver *getSolver() const {
        return solver_;
    }
    /** Returns the model used with this corrector. */
    virtual Model *getModel() const {
        return solver_->getModel();
    }

private:
    Solver *solver_;
};

} /* namespace abt */

#endif /* SOLVER_HISTORYCORRECTOR_HPP_ */
