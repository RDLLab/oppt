/** @file VectorState.hpp
 *
 * Defines the VectorState interface; currently this is the same as Vector, so this is just a
 * straight typedef.
 */
#ifndef SOLVER_VECTORSTATE_HPP_
#define SOLVER_VECTORSTATE_HPP_

#include "Vector.hpp"

namespace abt {
    /** The VectorState class currently has the same interface as Vector, because there are no
     * special methods required by State.
     */
    typedef Vector VectorState;
}

#endif /* SOLVER_VECTORSTATE_HPP_ */
