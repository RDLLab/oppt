/** @file HistoryEntry.hpp
 *
 * Contains the HistoryEntry class, which represents a single entry in a history sequence.
 *
 * The core functionality of the entry is to store a state, action, observation and reward
 * tuple (s, a, o, r), as per the ABT algorithm.
 */
#ifndef SOLVER_HISTORYENTRY_HPP_
#define SOLVER_HISTORYENTRY_HPP_

#include <cstdint>

#include <memory>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"                   // for Action
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "solvers/ABT/solver/abstract-problem/State.hpp"

#include "solvers/ABT/solver/abstract-problem/TransitionParameters.hpp"
#include "solvers/ABT/solver/changes/ChangeFlags.hpp"              // for ChangeFlags

namespace solvers
{
class ABT;
}

namespace oppt
{
class BeliefNode;
class ABTSolver;
}

namespace abt
{
class BeliefNode;
class HistorySequence;
class StateInfo;
class OPPTStatePool;

/** Represents a single entry in a sequence.
 *
 * A history entry is associated with a specific state (represented via StateInfo), and can also
 * own an action, transition parameters, and an observation; it also has an associated immediate
 * reward value - this represents the (s, a, o, r) tuple.
 *
 * If the action is null, this means that this entry is at the end of a history sequence, and
 * the transition parameters and observation should also be null. Otherwise, there will be
 * subsequent entries, and the resulting next state will be the state associated with the next
 * entry in the sequence.
 *
 * The entry has an ID, which is its position in the sequence (0 => first entry); it also keeps
 * a pointer to the history sequence that owns it, and the belief node this entry is associated
 * with.
 *
 * A history entry also keeps track of change flags, which mark off the ways in which this
 * entry has been affected by changes to the model.
 */
class HistoryEntry
{
public:
    friend class BasicSearchStrategy;
    friend class DefaultHistoryCorrector;
    friend class HistorySequence;
    friend class Simulator;
    friend class Solver;
    friend class TextSerializer;
    friend class OPPTStatePool;
    friend class oppt::BeliefNode;
    friend class oppt::ABTSolver;
    friend class solvers::ABT;

    /** The ID type of this HistoryEntry. This should be an integer type with at least 16 bits. */
    typedef uint16_t IdType;

    /** Constructs a new history entry, without a state!! */
    HistoryEntry();
    /** Constructs a new history entry with the given cumulative discount,
     * owning sequence, and entry ID.
     */
    HistoryEntry(HistorySequence* owningSequence, IdType entryId);
    /** Destroys this HistoryEntry. */
    virtual ~HistoryEntry();
    //_NO_COPY_OR_MOVE(HistoryEntry);

    /* ----------------- Simple getters ------------------- */
    /** Returns the id of this entry (0 = first entry in the sequence). */
    IdType getId() const;
    /** Returns the immediate reward for this entry. */
    FloatType getImmediateReward() const;
    /** Returns the cumulative discounted reward, starting at this entry. */
    FloatType getCumulativeReward() const;
    /** Returns the state associated with this history entry. */
    State const* getState() const;
    /** Returns the state info associated with this history entry. */
    StateInfo const* getStateInfo() const;
    /** Returns the action associated with this history entry. */
    Action const* getAction() const;
    /** Returns the observation associated with this history entry. */
    Observation const* getObservation() const;
    /** Returns the transition parameters associated with this
     * history entry.
     */
    TransitionParameters const* getTransitionParameters() const;
    /** Returns the belief node associated with this history entry. */
    BeliefNode* getAssociatedBeliefNode() const;

    void registerState(StateInfo* info);

protected:
    /* ----------------- Change flagging ------------------- */
    /**  Resets the changes that apply to this history entry. */
    void resetChangeFlags();
    /** Sets the given flags for this history entry. */
    void setChangeFlags(ChangeFlags flags);

    /* -------------- Registration methods ---------------- */
    /** Registers this entry as one of the particles of the given node. */
    virtual void registerNode(BeliefNode* node, bool addParticle = true);
    /** Registers this history entry as one of the particles that contains
     * the given state.
     * A value of nullptr will deregister this particle from that state.
     */


protected:
    /** The history sequence that owns this entry. */
    HistorySequence* owningSequence_;
    /** The belief node this entry is associated with. */
    BeliefNode* associatedBeliefNode_;

    /** The state information for this history entry. */
    StateInfo* stateInfo_;
    /** Action performed in this entry. */
    std::unique_ptr<Action> action_;
    /** Extra information about the transition, if any is needed. */
    std::unique_ptr<TransitionParameters> transitionParameters_;
    /** Observation received in this entry. */
    std::unique_ptr<Observation> observation_;
    /** Non-discounted reward. */
    FloatType immediateReward_;

    /** The id of the specific entry within the sequence. */
    IdType entryId_;
    /** The flags associated with current POMDP model updates. */
    ChangeFlags changeFlags_;
};
} /* namespace abt */

#endif /* SOLVER_HISTORYENTRY_HPP_ */
