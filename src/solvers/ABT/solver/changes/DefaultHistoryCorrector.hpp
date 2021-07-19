/** @file DefaultHistoryCorrector.hpp
 *
 * Provides a default, general-purpose implementation of a HistoryCorrector that will be able to
 * deal with any POMDP model.
 */
#ifndef SOLVER_DEFAULTHISTORYCORRECTOR_HPP_
#define SOLVER_DEFAULTHISTORYCORRECTOR_HPP_

#include "HistoryCorrector.hpp"

#include "solvers/ABT/solver/abstract-problem/heuristics/HeuristicFunction.hpp"

namespace abt {
class HistorySequence;

/** A default HistoryCorrector implementation which should work quite well regardless of the
 * specific problem.
 */
class DefaultHistoryCorrector: public abt::HistoryCorrector {
public:
    /** Constructs a new DefaultHistoryCorrector, which will be associated with the given Solver.
     *
     * The given heuristic will be used to estimate the value at the end of any sequence that
     * ends with a non-terminal state.
     */
    DefaultHistoryCorrector(Solver *solver, HeuristicFunction heuristic);
    virtual ~DefaultHistoryCorrector() = default;
    _NO_COPY_OR_MOVE(DefaultHistoryCorrector);

    /** Revises the given history sequence; returns false if the sequence previously took an
     * illegal action and hence requires continuation via the default search algorithm.
     */
    virtual bool reviseSequence(HistorySequence *sequence) override;
private:
    HeuristicFunction heuristic_;
};

} /* namespace abt */

#endif /* SOLVER_DEFAULTHISTORYCORRECTOR_HPP_ */
