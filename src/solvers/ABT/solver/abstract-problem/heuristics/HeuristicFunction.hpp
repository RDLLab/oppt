/** @file HeuristicFunction.hpp
 *
 * Contains a typedef for the "HeuristicFunction" type, which is represented via a function that
 * takes pointers to a history entry, state, and/or historical data, and returns a FloatType value.
 */
#ifndef SOLVER_HEURISTICFUNCTION_HPP_
#define SOLVER_HEURISTICFUNCTION_HPP_

#include <functional>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/State.hpp"

namespace abt {
class HistoricalData;
class HistoryEntry;

/** A typedef for the HeuristicFunction type.
 *
 * In order to allow for different types of heuristics, this heuristic takes three different
 * arguments via pointers. It is not necessary to use any or all of these arguments - you can
 * base a heuristic on whichever one you wish to.
 *
 * - Access to the history entry allows for heuristics based on the actual belief node, as
 *      the history entry keeps a pointer to the associated belief.
 * - Access to the state allows for "perfect information" heuristics, e.g. using the MDP value
 *      as a heuristic.
 * - Access to historical data allows simpler incorporation of history-based information into the
 *      heuristic value.
 */
typedef std::function<FloatType(HistoryEntry const *,
		State const *, HistoricalData const *)> HeuristicFunction;

} /* namespace abt */

#endif /* SOLVER_HEURISTICFUNCTION_HPP_ */
