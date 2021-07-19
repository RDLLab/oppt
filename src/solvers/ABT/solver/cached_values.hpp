/** @file cached_values.hpp
 *
 * Contains the classes used to store cached values for a belief node.
 */
#ifndef SOLVER_CACHED_VALUES_HPP_
#define SOLVER_CACHED_VALUES_HPP_

#include "oppt/global.hpp"

#include "solvers/ABT/solver/BeliefNode.hpp"

namespace abt {
/** An interface class so that cached values can easily be stored in a vector. */
class BaseCachedValue {
public:
    BaseCachedValue() = default;
    virtual ~BaseCachedValue() = default;
};

/** Represents a cached value associated with a specific belief node.
 *
 * This is done by wrapping a function that takes a pointer to a belief node and returns a value
 * of the value type together with a pointer to the specific belief node this cache is associated
 * with.
 *
 * This allows the value to be stored, and also recalculated on demand.
 */
template<typename T> class CachedValue : public BaseCachedValue {
public:
    /** Constructs a new CachedValue for the given node, which will use the given function in order
     * to recalculate the cahced value whenever it is needed.
     */
    CachedValue(BeliefNode const *node, std::function<T(BeliefNode const *)> f) :
                node_(node),
                function_(f),
                cache_() {
    }
    _NO_COPY_OR_MOVE(CachedValue);

    /** Updates the cache inside this cached value by using the stored function on the associated
     * belief node.
     */
    virtual void updateCache() {
        cache_ = function_(node_);
    }

    /** Returns the currently cached value. */
    virtual T getCache() {
        return cache_;
    }
private:
    /** The associated belief node. */
    BeliefNode const *node_;
    /** The function. */
    std::function<T(BeliefNode const *)> function_;
    /** The cached value. */
    T cache_;
};

/** A template specialization for caches of type unique_ptr<Point>.
 *
 * Since a unique_ptr can only be moved we need to make a new Point via the method Point::copy()
 */
template<> inline std::unique_ptr<abt::Point>
CachedValue<std::unique_ptr<abt::Point>>::getCache() {
    return cache_->copy();
}
} /* namespace abt */

#endif /* SOLVER_CACHED_VALUES_HPP_ */
