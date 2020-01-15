/** @file Action.hpp
 *
 * Defines the Action interface; currently this is the same as Point, so this is just a straight
 * typedef.
 */
#ifndef SOLVER_ACTION_HPP_
#define SOLVER_ACTION_HPP_

#include "solvers/ABT/solver/abstract-problem/Point.hpp"

namespace abt {
    /** Currently Action has no requirements over and above Point, so this is just a typedef. */
    typedef Point Action;
} /* namespace abt */

#endif /* SOLVER_ACTION_HPP_ */
