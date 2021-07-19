/** @file ObservationMapping.hpp
 *
 * Contains the abstract base class ObservationMapping.
 *
 * This class defines a mapping of observations following a previous belief and action to
 * subsequent beliefs in the belief tree - each entry in such a mapping is an edge in the tree,
 * which is represented by an ObservationMappingEntry.
 *
 * The purpose of this abstract class is to allow different observations to be grouped together in
 * customizable ways.
 */
#ifndef SOLVER_OBSERVATIONMAPPING_HPP_
#define SOLVER_OBSERVATIONMAPPING_HPP_

#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation
#include "ObservationMappingEntry.hpp"

namespace abt {
class ActionNode;
class BeliefNode;

/** An abstract base class that defines a mapping of observations to subsequent beliefs in the
 * belief tree.
 *
 * Each of these edges must also store the statistics for that edge - in this case, this only
 * consists of the visit count for that edge.
 */
class ObservationMapping {
public:
    /** Creates a new ObservationMapping, which will be owned by the given ActionNode. */
    ObservationMapping(ActionNode *owner) :
        owner_(owner) {
    }
    virtual ~ObservationMapping() = default;
    _NO_COPY_OR_MOVE(ObservationMapping);

    /* -------------- Association with an action node ---------------- */
    /** Returns the action node that owns this mapping. */
    ActionNode *getOwner() const {
        return owner_;
    }

    /* -------------- Access to and management of child nodes. ---------------- */
    /** Retrieves the belief node (if any) corresponding to this observation */
    virtual BeliefNode *getBelief(Observation const &obs) const = 0;
    /** Creates a new belief node for the given observation */
    virtual BeliefNode *createBelief(Observation const &obs) = 0;
    /** Returns the number of child nodes associated with this mapping. */
    virtual long getNChildren() const = 0;

    /** Deletes the given entry from this mapping, as well as the entire corresponding subtree. */
    virtual void deleteChild(ObservationMappingEntry const *entry) = 0;

    /* -------------- Retrieval of mapping entries. ---------------- */
    /** Returns a vector of all the entries in this mapping that have an associated child node. */
    virtual std::vector<ObservationMappingEntry const *> getChildEntries() const = 0;

    /** Returns the mapping entry associated with the given observation. */
    virtual ObservationMappingEntry *getEntry(Observation const &obs) = 0;
    /** Returns the mapping entry associated with the given observation. */
    virtual ObservationMappingEntry const *getEntry(Observation const &obs) const = 0;

    /* ------------- Methods for accessing visit counts. --------------- */
    /** Returns the total visit count among all observations. */
    virtual long getTotalVisitCount() const = 0;

private:
    /** The action node that owns this mapping. */
    ActionNode *owner_;
};
} /* namespace abt */

#endif /* SOLVER_OBSERVATIONMAPPING_HPP_ */
