/** @file default_rollout.hpp
 *
 * Defines a simple rollout strategy that queries the model for a history-based rollout action.
 * This is done via the StepGenerator and StepGeneratory factory classes in search_interface.hpp.
 */
#ifndef SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_
#define SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_

#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

namespace abt {
class HistoricalData;
class HistorySequence;
class Solver;

/** A StepGenerator implementation that simply queries the model for a rollout action at each
 * time step.
 */
class DefaultRolloutGenerator: public StepGenerator {
public:
    /** Makes a new DefaultRolloutGenerator, which will be associated with the given solver, and
     * will only take the specified maximum # of steps in depth.
     */
    DefaultRolloutGenerator(SearchStatus &status, Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutGenerator() = default;
    _NO_COPY_OR_MOVE(DefaultRolloutGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data, Action *action=nullptr) override;

private:
    /** The associated model, which will be queried to generate steps. */
    Model *model_;
    /** The maximum # of steps to take. */
    long maxNSteps_;
    /** The number of steps taken so far. */
    long currentNSteps_;
};


/** A factory class to create instances of DefaultRolloutGenerator. */
class DefaultRolloutFactory: public StepGeneratorFactory {
public:
    /** Creates a new rollout factory with the given solver, and the given max # of steps. */
    DefaultRolloutFactory(Solver *solver, long maxNSteps);
    virtual ~DefaultRolloutFactory() = default;
    _NO_COPY_OR_MOVE(DefaultRolloutFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) override;

private:
    /** The associated solver. */
    Solver *solver_;
    /** The maximum number of steps to take in a rollout. */
    long maxNSteps_;
};

} /* namespace abt */

#endif /* SOLVER_DEFAULTROLLOUTSTRATEGY_HPP_ */
