/** @file ObservationPool.hpp
 *
 * Defines the ObservationPool interface, which allows customization of how the mapping for each
 * individual action node is set up.
 *
 * Using a single class in this way allows certain aspects of the mappings to be stored globally,
 * e.g. to keep statistics that are shared across all of the mappings rather than stored on
 * a per-mapping basis.
 */
#ifndef SOLVER_OBSERVATIONPOOL_HPP_
#define SOLVER_OBSERVATIONPOOL_HPP_

#include "oppt/global.hpp"

namespace abt {
class ActionPool;
class ObservationMapping;
class Solver;

/** An interface class which is a factory for creating observation mappings.
 *
 * Using a central factory instance allows each individual mapping to interface with this single
 * instance; this allows shared statistics to be kept.
 */
class ObservationPool {
public:
    ObservationPool() = default;
    virtual ~ObservationPool() = default;
    /** Creates an observation mapping for the given action node. */
    virtual std::unique_ptr<ObservationMapping> createObservationMapping(ActionNode *owner) = 0;
};

} /* namespace abt */

#endif /* SOLVER_OBSERVATIONPOOL_HPP_ */
