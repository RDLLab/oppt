/** @file Point.hpp
 *
 * Defines the abstract Point class. This is primarily in the sense of point-set topology; in other
 * words, each point will be a member of some set, e.g. the set of actions for a given POMDP.
 */
#ifndef SOLVER_POINT_HPP_
#define SOLVER_POINT_HPP_

#include <cstddef>                      // for size_t

#include <string>
#include <sstream>
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream

#include "oppt/global.hpp"

namespace abt {
/** Represents a point within a certain set, in the sense of point-set topology.
 *
 * In order to manage states within the StatePool, an equals() method is required; this allows
 * the code to distinguish whether two points are the same.
 * WARNING: You should be very careful of this in the case of floating point values, due to
 *      the possibility of error. Simply using an error margin is also dangerous because this
 *      makes the equality operator intransitive!
 *
 * This class also has an optional distanceTo() method, which allows you to define a distance
 * metric on the set.
 */
class Point {
  public:
    Point() = default;
    virtual ~Point() = default;

    /** You must provide a copy() method for any subclass.
     *
     * This is crucial; we can't rely on copy constructors as we're using polymorphism, and we
     * want to avoid issues with slicing.
     */
    virtual std::unique_ptr<Point> copy() const = 0;

    /** Returns true iff this point is equal to the the given point. */
    virtual bool equals(Point const &otherPoint) const = 0;

    /** Returns a hash value for this point - should be consistent with equals() */
    virtual std::size_t hash() const = 0;

    /** Returns the distance from this point to the given point [optional]. */
    virtual FloatType distanceTo(Point const &/*otherPoint*/) const {
        return std::numeric_limits<FloatType>::infinity();
    }

    /** Prints this point in a human-readable way [optional] */
    virtual void print(std::ostream &/*os*/) const {};
    
    /** Serializes this point in a human-readable way [optional] */
    virtual void serialize(std::ostream &/*os*/, const std::string prefix="") const {};
};

/** Implements the  insertion operator via the virtual print method. */
inline std::ostream &operator<<(std::ostream &os, Point const &point) {
    point.print(os);
    return os;
}

/** Implements the equality operator for points based on the virtual equals method. */
inline bool operator==(Point const &s1, Point const &s2) {
    return s1.equals(s2); // && s2.equals(s1); (symmetry - assumed by default)
}

/** Implements the inequality operator for points based on the virtual equals method. */
inline bool operator!=(Point const &s1, Point const &s2) {
    return !s1.equals(s2); // || !s2.equals(s1); (symmetry - assumed by default)
}
} /* namespace abt */
#endif /* SOLVER_POINT_HPP_ */
