/** @file Histories.hpp
 *
 * Contains the Histories class, which represents a collection of history sequences.
 *
 * This class owns the associated sequences, which are stored in a vector of unique_ptr
 */
#ifndef SOLVER_HISTORIES_HPP_
#define SOLVER_HISTORIES_HPP_

#include <map>                          // for map
#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

namespace solvers{
    class ABT;
}

namespace oppt
{
class ABTSolver;
}

namespace abt
{
class HistoryEntry;
class HistorySequence;

/** Owns a collection of history sequences.
 *
 * The createSequence() method is the usual way to make a new history sequence, as it will
 * be owned by this Histories object.
 *
 * Individual sequences can also be removed outright using the deleteSequence() method. Note that
 * this is done by moving the sequence from the last position in the vector into the newly
 * deleted spot.
 *
 * Since the ID of a history sequence corresponds to its index in the vector, this means that
 * the ID of a history sequence is not guaranteed to remain constant. If you need to keep a
 * permanent reference to a sequence as long as it exists, just use a raw pointer to it.
 */
class Histories
{
public:
    friend class Solver;
    friend class TextSerializer;
    friend class oppt::ABTSolver;
    friend class solvers::ABT;

    /** Constructs an empty bundle of histories. */
    Histories();

    // Default destructor; copying and moving disallowed!
    virtual ~Histories() = default;
    _NO_COPY_OR_MOVE(Histories);

    /* ------------------- Retrieving sequences ------------------- */
    /** Returns the number of history sequences. */
    long getNumberOfSequences() const;
    /** Retrieves the history sequence with the given ID. */
    HistorySequence* getSequence(long seqId) const;


protected:
    /* ---------------- Adding / removing sequences  ---------------- */
    /** Resets the histories to be empty. */
    void reset();
    /** Adds a new history sequence. */
    virtual HistorySequence* createSequence();
    /** Deletes the given history sequence. */
    void deleteSequence(HistorySequence* sequence);

protected:
    /** A vector to hold all of the sequences in this collection. */
    std::vector<std::unique_ptr<HistorySequence>> sequencesById_;
};
} /* namespace abt */

#endif /* SOLVER_HISTORIES_HPP_ */
