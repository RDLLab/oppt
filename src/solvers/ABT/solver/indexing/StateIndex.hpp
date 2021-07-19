/** @file StateIndex.hpp
 *
 * Defines an interface allowing states to be indexed in a custom manner.
 *
 * Currently the only provided implementation of this interface is the RTree, which is a wrapper
 * for the RTree class within libspatialindex.
 *
 * The RTree-based approach should be sufficient for most purposes, but you can easily make your
 * own implementation of this interface if need be.
 */
#ifndef SOLVER_STATEINDEX_HPP_
#define SOLVER_STATEINDEX_HPP_

#include <memory>

#include "oppt/global.hpp"

namespace abt {
class StateInfo;

/** An interface class for storing states and looking them up in custom ways.
 *
 * Note that only a way of adding and removing states is mandatory; there are no required query
 * methods. This is because there could be many different kinds of query, depending on the
 * approach taken for indexing.
 *
 * You will need to define your own query methods if you implement this interface - your new
 * Model class can then interface with those query methods as needed.
 */
class StateIndex {
  public:
    StateIndex() = default;
    virtual ~StateIndex() = default;
    _NO_COPY_OR_MOVE(StateIndex);

    /** Resets the state index, making it empty. */
    virtual void reset() = 0;

    /** Adds the given state info to the index. */
    virtual void addStateInfo(StateInfo *stateInfo) = 0;

    /** Removes the given state info from the index. */
    virtual void removeStateInfo(StateInfo *stateInfo) = 0;
};
} /* namespace abt */

#endif /* SOLVER_STATEINDEX_HPP_ */
