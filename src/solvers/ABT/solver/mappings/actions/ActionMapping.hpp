/** @file ActionMapping.hpp
 *
 * Contains the abstract base class ActionMapping.
 *
 * In essence, this class defines, for a single belief node, the mapping of actions to
 * (belief, action) edges in the belief tree - each of these is represented by an
 * ActionMappingEntry.
 *
 * The purpose of this abstract class is to allow different actions to be grouped together in
 * customizable ways.
 */
#ifndef SOLVER_ACTIONMAPPING_HPP_
#define SOLVER_ACTIONMAPPING_HPP_

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

#include "ActionMappingEntry.hpp"

namespace abt {
class ActionMappingEntry;
class ActionNode;
class BeliefNode;

/** An abstract base class that defines a mapping of the actions of a belief to (belief, action)
 * edges in the belief tree; these edges are represented by the ActionMappingEntry class.
 *
 * Each of these edges must also store the statistics for that edge - most notably, the visit
 * count and estimated Q-value.
 */
class ActionMapping {
public:
    /** Creates a new ActionMapping, which will be owned by the given belief node. */
    ActionMapping(BeliefNode *owner) :
        owner_(owner) {
    }
    virtual ~ActionMapping() = default;
    _NO_COPY_OR_MOVE(ActionMapping);

    /* -------------- Association with a belief node ---------------- */
    /** Returns the belief node that owns this mapping. */
    BeliefNode *getOwner() const {
        return owner_;
    }

    /* -------------- Access to and management of child nodes. ---------------- */
    /** Retrieves the action node (if any) corresponding to the given action. */
    virtual ActionNode *getActionNode(Action const &action) const = 0;
    /** Creates a new action node for the given action. */
    virtual ActionNode *createActionNode(Action const &action) = 0;
    /** Returns the number of child action nodes owned by this mapping. */
    virtual long getNChildren() const = 0;

    /** Deletes the child in the given entry, as well as the entire corresponding subtree. */
    virtual void deleteChild(ActionMappingEntry const *entry) = 0;

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns all entries in this mapping that have a child node associated with them. */
    virtual std::vector<ActionMappingEntry const *> getChildEntries() const = 0;


    /** Returns the number of entries in this mapping with a nonzero visit count.
     *
     * This is different to the the number of child nodes, because sometimes there will be child
     * nodes with zero visit counts due to deleted entries, and sometimes there will be nonzero
     * visit counts without an associated action node due to initialization with nonzero values.
     */
    virtual long getNumberOfVisitedEntries() const = 0;

    /** Returns a vector of all of the visited entries in this mapping.
     *
     * Some of those entries might have null action nodes if the visit counts were initialized
     * with nonzero values.
     */
    virtual std::vector<ActionMappingEntry const *> getVisitedEntries() const = 0;

    /** Returns the mapping entry associated with the given action, or nullptr if there is none. */
    virtual ActionMappingEntry *getEntry(Action const &action) = 0;

    /** Returns the mapping entry associated with the given action, or nullptr if there is none.
     *
     * This version returns a pointer-to-const, which is useful if you want to enforce
     * immutability.
     */
    virtual ActionMappingEntry const *getEntry(Action const &action) const = 0;

    /* ------------------ Methods for unvisited actions ------------------- */
    /** Returns the next unvisited action to be tried for this node, or nullptr if there are no
     * more unvisited actions (that are legal).
     */
    virtual std::unique_ptr<Action> getNextActionToTry() = 0;

    /* -------------- Retrieval of general statistics. ---------------- */
    /** Returns the total number of times children of this mapping have been visited. */
    virtual long getTotalVisitCount() const = 0;

private:
    /** The belief node that owns this mapping. */
    BeliefNode *owner_;
};
} /* namespace abt */

#endif /* SOLVER_ACTIONMAPPING_HPP_ */
