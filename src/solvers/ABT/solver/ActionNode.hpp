/** @file ActionNode.hpp
 *
 * Contains the ActionNode class, which represents a belief-action pair (i.e. the part of the
 * belief tree corresponding to taking a given action from a specific belief.
 */
#ifndef SOLVER_ACTIONNODE_HPP_
#define SOLVER_ACTIONNODE_HPP_

#include <memory>                       // for unique_ptr
#include <utility>                      // for pair
#include <vector>                       // for vector

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"                   // for Action
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation

#include "solvers/ABT/solver/mappings/observations/ObservationMapping.hpp"

namespace abt {
class ActionMappingEntry;
class BeliefNode;
class ObservationMapping;
class Solver;

/** The base level interface used by the solver to store the results of taking a specific action
 * from a specific belief.
 *
 * For purposes of customizability most of the work is done in the ActionMapping and
 * ObservationMapping interfaces, which allow for custom approaches to implementing those mappings.
 *
 * This class contains only two fields; a back-pointer to the ActionMappingEntry that owns this
 * action node (and stores relevant statistics), and an ObservationMapping, which is owned by
 * this ActionNode, and stores information about the observations branching out of this node.
 */
class ActionNode {
    friend class BeliefNode;
    friend class TextSerializer;

  public:
    /** Constructs an action node without an observation mapping! */
    ActionNode();
    /** Constructs an action node with the given entry as its parent. */
    ActionNode(ActionMappingEntry *parentEntry);

    // Default destructor; copying and moving disallowed!
    ~ActionNode();
    _NO_COPY_OR_MOVE(ActionNode);


    /* -------------------- Tree-related getters  ---------------------- */
    /** Returns the observation mapping for this node. */
    ObservationMapping *getMapping() const;
    /** Returns the parent entry for this node. */
    ActionMappingEntry *getParentEntry() const;
    /** Returns the parent belief node of this action node. */
    BeliefNode *getParentBelief() const;
    /** Returns the child corresponding to the given observation, based on
     * sufficient proximity.
     */
    BeliefNode *getChild(Observation const &obs) const;

  private:
    /* -------------------- Tree-related setters  ---------------------- */
    /** Sets the mapping for this node. */
    void setMapping(std::unique_ptr<ObservationMapping> mapping);

    /* -------------------- Tree-related methods  ---------------------- */
    /** Adds a child with the given observation, creating a new belief node if
     * necessary.
     */
    std::pair<BeliefNode *, bool> createOrGetChild(Solver *solver,
            Observation const &obs);

  private:
    /** The mapping entry that owns this action node. */
    ActionMappingEntry *parentEntry_;
    /** A mapping to store the observations that branch out of this node, and their associated
     * entries, statistics, and subtrees.
     */
    std::unique_ptr<ObservationMapping> observationMap_;
};
} /* namespace abt */

#endif /* SOLVER_ACTIONNODE_HPP_ */
