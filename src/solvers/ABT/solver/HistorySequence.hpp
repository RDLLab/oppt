/** @file HistorySequence.hpp
 *
 * Contains the HistorySequence class, which represents a single history sequence.
 *
 * For the most part, a history sequence is just a vector of history entries; it also stores
 * the starting index and ending index of any changes that affect this sequence, as well as
 * the collective types of these changes.
 */
#ifndef SOLVER_HISTORYSEQUENCE_HPP_
#define SOLVER_HISTORYSEQUENCE_HPP_

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector

#include "oppt/global.hpp"

#include "solvers/ABT/solver/HistoryEntry.hpp"

#include "solvers/ABT/solver/changes/ChangeFlags.hpp"               // for ChangeFlags

#include "solvers/ABT/solver/abstract-problem/Action.hpp"                   // for Action
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"

namespace solvers
{
class ABT;
}

namespace oppt
{
class ABTSolver;
}


namespace abt
{
class BeliefNode;
class BeliefTree;
class StateInfo;
class OPPTStatePool;
class HistorySequence;

/** Represents a single history sequence.
 *
 * The sequence owns its entries, which are stored in a vector of unique_ptr<HistoryEntry>.
 *
 * The sequence also keeps track of the first index and last index for entries that have been
 * affected by changes, as well as the logical disjunction (or) of all changes that affect the
 * entries in the sequence.
 */
class HistorySequence
{
public:
    friend class BasicSearchStrategy;
    friend class DefaultHistoryCorrector;
    friend class Histories;
    friend class Simulator;
    friend class Solver;
    friend class TextSerializer;
    friend class OPPTStatePool;
    friend class oppt::ABTSolver;
    friend class solvers::ABT;

    /** Constructs an empty history sequence, with no ID assigned. */
    HistorySequence();
    /** Constructs an empty history sequence, assigning the given ID. */
    HistorySequence(long id);

    // Default destructor; copying and moving disallowed!
    virtual ~HistorySequence();
    //_NO_COPY_OR_MOVE(HistorySequence);

    /* ------------------ Simple getters ------------------- */
    /** Returns the ID of this sequence. */
    long getId() const;
    /** Returns the length of this sequence. */
    long getLength() const;
    /** Returns the history entry in this sequence with the given ID. */
    HistoryEntry* getEntry(HistoryEntry::IdType entryId) const;
    /** Returns the first entry in this sequence. */
    HistoryEntry* getFirstEntry() const;
    /** Returns the last entry in this sequence. */
    HistoryEntry* getLastEntry() const;
    /** Returns the states in this sequence as a vector. */
    std::vector<State const*> getStates() const;

protected:
    /* ----------- Methods to add or remove history entries ------------- */
    /** Erases all of the entries in this sequence, starting from firstEntryId. */
    void erase(HistoryEntry::IdType firstEntryId = 0);
    /** Adds a new entry to this sequence, and returns a pointer to it. */
    virtual HistoryEntry* addEntry();

    /* -------------- Change flagging methods ---------------- */
    /** Resets the changes for this sequence and all its entries. */
    void resetChangeFlags();
    /** Sets the given entry as having the given flags. */
    void setChangeFlags(HistoryEntry::IdType entryId, ChangeFlags flags);
    /** Sets the given change flags for this sequence. */
    void setChangeFlags(ChangeFlags flags);
    /** Resets the range affected indices for this sequence. */
    void resetAffectedIndices();
    /** Adds the given index as one of those affected by changes. */
    void addAffectedIndex(HistoryEntry::IdType entryId);

protected:
    /** The ID of this sequence. */
    long id_;

    /** The actual sequence of history entries. */
    std::vector<std::unique_ptr<HistoryEntry>> entrySequence_;

    /** The start and end of where this sequence is affected by changes. */
    long startAffectedIdx_, endAffectedIdx_;
    /** The types of changes that have affected this sequence. */
    ChangeFlags changeFlags_;
};
} /* namespace abt */

#endif /* SOLVER_HISTORYSEQUENCE_HPP_ */
