/** @file ActionPool.hpp
 *
 * Defines the ActionPool interface, which allows customization of how the mapping for each
 * individual belief node is set up.
 *
 * Using a single class in this way allows certain aspects of the mappings to be stored globally,
 * e.g. to keep statistics that are shared across all of the mappings rather than stored on
 * a per-mapping basis.
 */
#ifndef SOLVER_ACTIONPOOL_HPP_
#define SOLVER_ACTIONPOOL_HPP_

#include <memory>                       // for unique_ptr

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"              // for Action
#include "solvers/ABT/solver/BeliefNode.hpp"

namespace abt {
class ActionMapping;
class ActionNode;
class BeliefNode;
class HistoricalData;
class ObservationPool;
class Solver;

/** An interface class which is a factory for creating action mappings.
 *
 * Using a central factory instance allows each individual mapping to interface with this single
 * instance; this allows shared statistics to be kept.
 */
class ActionPool {
public:
    ActionPool() = default;
    virtual ~ActionPool() = default;
    /** Creates an action mapping for the given belief node. */
    virtual std::unique_ptr<ActionMapping> createActionMapping(BeliefNode *node) = 0;
};
} /* namespace abt */

#endif /* SOLVER_ACTIONPOOL_HPP_ */
