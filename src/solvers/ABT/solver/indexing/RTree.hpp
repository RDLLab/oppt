/** @file indexing/RTree.hpp
 *
 * Contains the RTree class, which is an implementation of the StateIndex interface that functions
 * as a thin wrapper for the RTree class of libspatialindex.
 */
#ifndef SOLVER_RTREE_HPP_
#define SOLVER_RTREE_HPP_

#include <limits>
#include <memory>
#include <string>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>

#include "oppt/global.hpp"

#include "StateIndex.hpp"

namespace abt {
class SpatialIndexVisitor;
class StateInfo;
class StatePool;

/** A thin wrapper class that used the RTree class of libspatialindex to allow for range-based
 * state queries.
 *
 * The only constructor parameter is nSDim, the number of dimensions for a state vector.
 *
 * The core query method is boxQuery(), which takes a visitor and two state vectors (which must
 * clearly each have nSDim as their # of elements), and passes each StateInfo encountered during
 * the query onto the visitor.
 */
class RTree : public StateIndex {
  public:
    /** Constructs a new RTree with the given number of state dimensions. */
    RTree(unsigned int nSDim);
    virtual ~RTree() = default;
    _NO_COPY_OR_MOVE(RTree);

    /** Resets this RTree, making it empty. */
    virtual void reset() override;

    /** Adds the given StateInfo to this RTree.
     * NOTE: the same StateInfo / same ID must not have been added since the last reset.
     */
    virtual void addStateInfo(StateInfo *stateInfo) override;

    /** Removes the given StateInfo from this RTree.
     * NOTE: Undefined behaviour results if this StateInfo hasn't been added since after the last
     * reset.
     */
    virtual void removeStateInfo(StateInfo *stateInfo) override;

    /** Performs a range query on the RTree. All StateInfo that are encountered in the range of
     * the query will be passed on to the given visitor.
     */
    virtual void boxQuery(SpatialIndexVisitor &visitor,
            std::vector<FloatType> lowCorner,
            std::vector<FloatType> highCorner);

  private:
    /** The number of state dimensions for this RTree. */
    unsigned int nSDim_;
    /** The properties of the RTree. */
    std::unique_ptr<Tools::PropertySet> properties_;
    /** A storage manager for the RTree. */
    std::unique_ptr<SpatialIndex::IStorageManager> storageManager_;
    /** The RTree itself. */
    std::unique_ptr<SpatialIndex::ISpatialIndex> tree_;
};
} /* namespace abt */

#endif /* SOLVER_RTREE_HPP_ */
