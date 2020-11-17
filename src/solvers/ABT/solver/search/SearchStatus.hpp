/** @file SearchStatus.hpp
 *
 * Defines a basic enumeration of the different states the search for an individual history
 * could be in.
 */
#ifndef SOLVER_SEARCHSTATUS_HPP_
#define SOLVER_SEARCHSTATUS_HPP_

namespace abt {

/** An enumeration of possible states for searching an individual history. */
enum class SearchStatus : long {
    UNINITIALIZED, // Not yet set up - could indicate a failure to meet preliminary conditions.
    INITIAL, // Ready to go.
    OUT_OF_STEPS, // The step generator is out of steps -
    FINISHED, // The history is finished now (i.e. a terminal state, or a non-terminal state
              // for which the heuristic value has already been calculated).
    ERROR // An error occurred in the search (e.g. UCB had no legal actions to choose from)
};
} /* namespace abt */

#endif /* SOLVER_SEARCHSTATUS_HPP_ */
