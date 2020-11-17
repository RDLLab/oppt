/** @file ucb_search.hpp
 *
 * Contains the necessary classes for a UCB-based search strategy; this is done via an
 * implementation of StepGenerator and StepGeneratorFactory; the latter can then be wrapped
 * inside a BasicSearchStrategy.
 */
#ifndef SOLVER_UCB_SEARCH_HPP_
#define SOLVER_UCB_SEARCH_HPP_

#include "solvers/ABT/solver/search/SearchStatus.hpp"
#include "solvers/ABT/solver/search/search_interface.hpp"

namespace abt {
/** A generator for steps that uses UCB to select actions.
 *
 * The action will be selected using UCB as long as the last action has been tried before; once
 * an action that has never been tried before is encountered, the search will terminate.
 */
class UcbStepGenerator : public StepGenerator {
public:
    /** Creates a new UcbStepGenerator associated with the given solver, and using the given
     * value for the UCB exploration coefficient.
     */
    UcbStepGenerator(SearchStatus &status, Solver *solver, FloatType explorationCoefficient);
    ~UcbStepGenerator() = default;
    _NO_COPY_OR_MOVE(UcbStepGenerator);

    virtual Model::StepResult getStep(HistoryEntry const *entry,
            State const *state, HistoricalData const *data, Action *action=nullptr) override;

private:
    /** The model to use to generate next steps. */
    Model *model_;
    /** The exploration coefficient for UCB. */
    FloatType explorationCoefficient_;

    /** True iff the last action selected hadn't been tried before. */
    bool choseUnvisitedAction_;
};

/** A factory class for generating instances of UcbStepGenerator. */
class UcbStepGeneratorFactory: public StepGeneratorFactory {
public:
    /** Creates a new factory associated with the given solver, and with the given UCB exploration
     * coefficient.
     */
    UcbStepGeneratorFactory(Solver *solver, FloatType explorationCoefficient);
    virtual ~UcbStepGeneratorFactory() = default;
    _NO_COPY_OR_MOVE(UcbStepGeneratorFactory);

    virtual std::unique_ptr<StepGenerator> createGenerator(SearchStatus &status,
            HistoryEntry const *entry, State const *state, HistoricalData const *data) override;
private:
    /** The associated solver. */
    Solver *solver_;
    /** The exploration coefficient for UCB. */
    FloatType explorationCoefficient_;
};

} /* namespace abt */

#endif /* SOLVER_UCB_SEARCH_HPP_ */
