/** @file ActionMappingEntry.hpp
 *
 * Defines the ActionMappingEntry interface class,
 *
 * This interface defines the core methods that are required for a mapping entry to function
 * properly; in particular, the update() method is of crucial importance - it is used to update
 * the visit count and/or q-value estimate for this edge of the
 */
#ifndef SOLVER_ACTIONMAPPINGENTRY_HPP_
#define SOLVER_ACTIONMAPPINGENTRY_HPP_

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

namespace abt {
class ActionNode;
class ActionMapping;

/** An interface that represents a (belief, action) edge in the belief tree.
 *
 * There are two core pieces of functionality - a number of getter methods returning various
 * properties of this edge, as well as, more importantly, two key mutator methods:
 * update(), which updates the visit count and/or Q-value for this edge, and
 * setLegal(), which allows this edge to be made legal or illegal.
 */
class ActionMappingEntry {
public:
    ActionMappingEntry() = default;
    virtual ~ActionMappingEntry() = default;

    /** Returns the mapping this entry belongs to. */
    virtual ActionMapping *getMapping() const = 0;
    /** Returns the action for this entry. */
    virtual std::unique_ptr<Action> getAction() const = 0;
    /** Returns the action node for this entry. */
    virtual ActionNode *getActionNode() const = 0;
    /** Returns the visit count for this entry. */
    virtual long getVisitCount() const = 0;
    /** Returns the total estimated Q-value for this entry. */
    virtual FloatType getTotalQValue() const = 0;
    /** Returns the mean estimated Q-value for this entry. */
    virtual FloatType getMeanQValue() const = 0;
    /** Returns true iff this action is legal (illegal => totally ignored). */
    virtual bool isLegal() const = 0;

    /** Updates this action, by adding the given number of visits and the
     * given change in the total q-value.
     *
     * Returns true if and only if the q value of the action changed.
     */
    virtual bool update(long deltaNVisits, FloatType deltaTotalQ) = 0;

    /** Sets the legality of this action - this determines whether or not it will be taken in the
     * course of *future* searches.
     *
     * In and of itself, making this action illegal will not delete previous histories that have
     * already taken this action. In order to achieve this the associated history entries also
     * need to be marked for updating via the model-changing interface.
     */
    virtual void setLegal(bool legal) = 0;

    /** Returns a to give extended information about the entry.
     *
     * This can be used for debugging or performance analysis.
     *
     * The default implementation returns an empty string.
     */
    virtual std::string getMarker() const { return ""; }
};
} /* namespace abt */

#endif /* SOLVER_ACTIONMAPPINGENTRY_HPP_ */
