/** @file DiscretizedPoint.hpp
 *
 * Defines a point within a discretized space. In addition to the default point methods, this class
 * requires each point to be able to return its associated bin number.
 */
#ifndef SOLVER_DISCRETIZED_POINT_HPP_
#define SOLVER_DISCRETIZED_POINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Point.hpp"

namespace abt {
/** Represents a point within a discretized space.
 *
 * This can be used to easily implement a simple, discrete state space by simply associating a
 * unique ID with each individual state.
 *
 * However, there need not be only one point in each bin; a bin can have as many or as few points
 * as you prefer.
 *
 * For convenience, the standard implementations for the equals(), hash() and print() are
 * implemented simply by using the ID. If you want something more sophisticated, override them.
 */
class DiscretizedPoint : public Point {
  public:
    DiscretizedPoint() = default;
    virtual ~DiscretizedPoint() = default;

    /** Returns the bin number associated with this point. */
    virtual long getBinNumber() const = 0;

    // Simple implementations usinng just the bin number.
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;
};
} /* namespace abt */

#endif /* SOLVER_DISCRETIZED_POINT_HPP_ */
