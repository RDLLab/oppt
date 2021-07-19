/** @file SpatialIndexVisitor.hpp
 *
 * Defines the SpatialIndexVisitor class, which provides a visitor pattern-based approach to
 * deal with the results of querying states within an RTree.
 */
#ifndef SOLVER_SPATIALINDEXVISITOR_HPP_
#define SOLVER_SPATIALINDEXVISITOR_HPP_

#include <vector>

#include <spatialindex/SpatialIndex.h>

#include "oppt/global.hpp"

namespace abt {
class StateInfo;
class StatePool;

/** An abstract class which implements the IVisitor interface from libspatialindex in order
 * to provide a visitor pattern approach to querying states within an RTree.
 *
 * The mandatory virtual method is the method visit(), which allows the visitor pattern to
 * handle each state visited during the query.
 */
class SpatialIndexVisitor: public SpatialIndex::IVisitor {
  public:
    /** Constructs a new SpatialIndexVisitor, which will query the given StatePool in order to
     * access the StateInfo instances associated with the entries in the spatial index.
     */
    SpatialIndexVisitor(StatePool *statePool);
    virtual ~SpatialIndexVisitor();
    _NO_COPY_OR_MOVE(SpatialIndexVisitor);

    /** The method used when visiting an individual node - we don't use this one. */
    virtual void visitNode(const SpatialIndex::INode &node) override;
    /** This is the key method of IVisitor we need to override in order to implement the
     * functionality we need.
     */
    virtual void visitData(const SpatialIndex::IData &data) override;

    /** The method used when visiting a vector of data - we don't use this one as we visit one
     * data entry at a time.
     */
    virtual void visitData(std::vector<const SpatialIndex::IData*> &v) override;

    /** A method following the visitor design patter, allowing each StateInfo to be processed. */
    virtual void visit(StateInfo *info) = 0;

    /** Returns the StatePool associated with this SpatialIndexVisitor. */
    StatePool *getStatePool() const;

  private:
    /** The StatePool used by this visitor. */
    StatePool *statePool_;
};

} /* namespace abt */

#endif /* SOLVER_SPATIALINDEXVISITOR_HPP_ */
