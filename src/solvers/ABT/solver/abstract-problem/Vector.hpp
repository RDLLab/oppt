/** @file Vector.hpp
 *
 * An abstract extension of the Point class for points that can be easily be represented as
 * real vectors.
 *
 * This is primarily used to provide an interface for storing states (via VectorState, which is
 * currently just typedef of Vector) within an RTree. This allows for efficient lookup via
 * range-based queries.
 */
#ifndef SOLVER_VECTOR_HPP_
#define SOLVER_VECTOR_HPP_

#include <cstddef>                      // for size_t

#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream
#include <vector>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/abstract-problem/Point.hpp"

namespace abt {
/** An abstract class for states that can easily be represented as real vectors.
 *
 * This is done via a virtual asVector() method; this is required in order to be able to store
 * states within the RTree.
 */
class Vector: public abt::Point {
public:
    Vector() = default;
    virtual ~Vector() = default;

    /** Returns this vector in the form of an std::vector<FloatType>.
     *
     * This allows for vector-like functionality; we don't implement it directly as we are being
     * storage-agnostic here.
     */
    virtual std::vector<FloatType> asVector() const = 0;

    // copy() remains unimplemented because we have no data!

    // The default implementation for distance uses a Euclidean metric.
    virtual FloatType distanceTo(Point const &otherPoint) const override;

    // We will give default implementations for these methods using asVector().
    virtual bool equals(Point const &otherPoint) const override;
    virtual std::size_t hash() const override;
    virtual void print(std::ostream &os) const override;
};

} /* namespace abt */

#endif /* SOLVER_VECTOR_HPP_ */
