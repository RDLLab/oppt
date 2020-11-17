/** @file ObservationMappingEntry.hpp
 *
 * Defines the ObservationMappingEntry interface, which defines the core methods for an edge in
 * the tree between an ActionNode and a subsequent BeliefNode.
 *
 * In particular, this involves storing the visit count for this specific edge in the tree.
 */
#ifndef SOLVER_OBSERVATIONMAPPINGENTRY_HPP_
#define SOLVER_OBSERVATIONMAPPINGENTRY_HPP_

#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Observation.hpp"              // for Observation

namespace abt {
class ActionNode;
class BeliefNode;
class ObservationMapping;

/** An interface that represents an edge in the belief tree between an action node and a
 * subsequent belief node; this interface is provided so that observations can be grouped together
 * in custom ways.
 *
 * Conceptually, this corresponds to a (belief, action, observation) triplet (b, a, o), or,
 * equivalently, it can be seen as the parent edge of the resulting belief (b').
 *
 * Apart from grouping observations together, the primary purpose of this entry is to store
 * a visit count - i.e. the number of times this edge has been visited during searching.
 */
class ObservationMappingEntry {
public:
    ObservationMappingEntry() = default;
    virtual ~ObservationMappingEntry() = default;

    /** Returns the mapping this entry belongs to. */
    virtual ObservationMapping *getMapping() const = 0;
    /** Returns the observation for this entry. */
    virtual std::unique_ptr<Observation> getObservation() const = 0;
    /** Returns the belief node for this entry. */
    virtual BeliefNode *getBeliefNode() const = 0;
    /** Returns the visit count for this entry. */
    virtual long getVisitCount() const = 0;

    /** Updates the visit count for this observation. */
    virtual void updateVisitCount(long deltaNVisits) = 0;
};
} /* namespace abt */

#endif /* SOLVER_OBSERVATIONMAPPINGENTRY_HPP_ */
