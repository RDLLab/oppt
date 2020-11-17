/** @file StatePool.hpp
 *
 * Contains the StatePool class, which stores all of the states that have been encountered by
 * the solver.
 *
 * The StatePool's primary purpose is to provide a way to mark which states have been affected by
 * changes, and to retrieve the set of affected states.
 *
 * The StatePool also stores a StateIndex, the main purpose of which is to provide an efficient
 * way to look up states.
 */
#ifndef SOLVER_STATEPOOL_HPP_
#define SOLVER_STATEPOOL_HPP_

#include <cstddef>                      // for size_t

#include <map>                          // for multimap
#include <memory>                       // for unique_ptr
#include <unordered_map>                // for unordered_map
#include <unordered_set>                // for unordered_set
#include <vector>                       // for vector

#include "oppt/global.hpp"

#include "StateInfo.hpp"                // for StateInfo

#include "abstract-problem/State.hpp"                    // for State, operator==
#include "changes/ChangeFlags.hpp"               // for ChangeFlags
#include "indexing/StateIndex.hpp"

namespace abt {
class Model;
class StateIndex;

/** A class to hold all of the states encountered by the ABT algorithm.
 *
 * The key functionality is to allow states to be marked for changes; this is done via the
 * setChangeFlags() method.
 *
 * The set of all states affected by changes can be accessed via getAffectedStates().
 *
 * The pool allows states to be looked up by ID; more complicated lookup operations
 * (typically based on spatial coordinates) should be handled via the StateIndex, which can be
 * retrieved via getStateIndex().
 */
class StatePool {
    friend class Solver;
    friend class TextSerializer;

  public:
    /** A hash function that operates on states via pointers to them. */
    struct Hash {
        /** Returns the hash value for the pointed-to state. */
        std::size_t operator()(State const *state) const {
            return state->hash();
        }
    };
    /** An equality test that operates on states via pointers to them. */
    struct EqualityTest {
        /** Returns true if the two pointed-to stated are equal. */
        bool operator()(State const *s1, State const *s2) const {
            return *s1 == *s2;
        }
    };
    /** An unordered map for looking up the StateInfo for a given State. */
    typedef std::unordered_map<State const *, StateInfo *, Hash, EqualityTest> StateInfoMap;

    /** Constructs a new StatePool with the given StateIndex. */
    StatePool(std::unique_ptr<StateIndex> stateIndex);
    virtual ~StatePool();
    _NO_COPY_OR_MOVE(StatePool);

    /* ------------------ Simple getters ------------------- */
    /** Returns the StateInfo for the given state, or nullptr if there is no info. */
    virtual StateInfo *getInfo(State const *state) const;
    /** Returns the info at the given ID.
     * NOTE: it is a prerequisite that 0 <= id < getNumberOfStates(); otherwise memory access
     * violations will result!
     */
    virtual StateInfo *getInfoById(long id) const;
    /** Returns the StateIndex used by this pool. */
    StateIndex *getStateIndex() const;
    /** Returns the number of states in this pooll. */
    virtual long getNumberOfStates() const;

    /* ------------------ State lookup ------------------- */
    /** Returns a StateInfo for the given state, creating a new one if there wasn't one already. */
    virtual StateInfo *createOrGetInfo(std::unique_ptr<State>);

    /* ---------------- Flagging of states with changes ----------------- */
    /** Resets the change flags for the given StateInfo, and removes it from the set of affected
     * states. */
    void resetChangeFlags(StateInfo *stateInfo);
    /** Resets the change flags for the given StateInfo. */
    void setChangeFlags(StateInfo *stateInfo, ChangeFlags flags);
    /** Clears the change flags for all states that are currently marked as affected, and clears
     * the set of affected states.
     */
    void resetAffectedStates();
    /** Returns the current set of affected states. */
    std::unordered_set<StateInfo *> getAffectedStates() const;
    
    virtual std::vector<State const*> getStates() const;
    
protected:
    /** The StateIndex used by this pool. */
    std::unique_ptr<StateIndex> stateIndex_;

    /** The set of states currently marked as affected by changes. */
    std::unordered_set<StateInfo *> changedStates_;

  private:
    /* ------------------ Mutators for the pool ------------------- */
    /** Takes possession of the given StateInfo and adds it to the pool. */
    StateInfo *add(std::unique_ptr<StateInfo> stateInfo);

  private:
    /** An unordered mapping of states to their associated StateInfo. */
    StateInfoMap stateInfoMap_;
    /** The vector that actually stores the StateInfo; also allows lookup of states by ID. */
    std::vector<std::unique_ptr<StateInfo>> statesByIndex_;    
};
} /* namespace abt */

#endif /* SOLVER_STATEPOOL_HPP_ */
