/** @file RolloutHeuristic.hpp
 *
 * Defines a simple heuristic that uses a rollout-based approach to estimate the value at the end
 * of a history sequence.
 */
#ifndef SOLVER_ROLLOUTHEURISTIC_HPP_
#define SOLVER_ROLLOUTHEURISTIC_HPP_

#include <memory>

#include "oppt/global.hpp"

#include "HeuristicFunction.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

namespace abt {
class HistoryEntry;

/** A simple heuristic that uses a rollout to estimate the value of a history.
 *
 * Unlike rollouts within the ABT algorithm, the steps rollouts will not be stored as history
 * entries, and new belief nodes will not be created for them. This avoids a lot of overhead and
 * allows for rollouts that behave like standard MCTS rollouts.
 *
 * A step generator is used to generate the steps for the heuristic, and a second, different
 * heuristic (other than this one) is applied to the final state if it is non-terminal.
 *
 * It is important to note that using this kind of rollout-based heuristic makes it more difficult
 * to determine which heuristic values need to be recalculated when changes occur.
 */
class RolloutHeuristic {
public:
    /** Constructs a new rollout-based heuristic for the given model, and using the given
     * factory to generate new */
    RolloutHeuristic(Model *model, std::unique_ptr<StepGeneratorFactory> factory,
            HeuristicFunction heuristic);
    ~RolloutHeuristic() = default;
    _NO_COPY_OR_MOVE(RolloutHeuristic);

    /** Uses a rollout to generate a heuristic value for the given entry, state and data. */
    FloatType getHeuristicValue(HistoryEntry const *entry,
            State const *state, HistoricalData const *data);

    /** Returns this heuristic as an actual HeuristicFunction. */
    HeuristicFunction asFunction();
private:
    Model *model_;
    std::unique_ptr<StepGeneratorFactory> factory_;
    HeuristicFunction heuristic_;
};
} /* namespace abt */

#endif /* SOLVER_ROLLOUTHEURISTIC_HPP_ */
