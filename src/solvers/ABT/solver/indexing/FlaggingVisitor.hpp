/** @file FlaggingVisitor.hpp
 *
 * Defines the FlaggingVisitor class, which is useful for flagging affected states that are looked
 * up via an RTree-based StateIndex.
 */
#ifndef SOLVER_FLAGGINGVISITOR_HPP_
#define SOLVER_FLAGGINGVISITOR_HPP_

#include "solvers/ABT/solver/changes/ChangeFlags.hpp"

#include "SpatialIndexVisitor.hpp"

#include "oppt/global.hpp"

namespace abt {
class StateInfo;
class StatePool;

/** An implementation of SpatialIndexVisitor that flags any StateInfo it encounters with the given
 * flags.
 *
 * This is useful for purposes such as marking a range of states as deleted.
 */
class FlaggingVisitor: public abt::SpatialIndexVisitor {
  public:
    /** Constructs a new FlaggingVisitor which is associated with the given StatePool, and will
     * set the given flags in each StateInfo it encounters.
     */
    FlaggingVisitor(StatePool *pool, ChangeFlags flagsToSet);
    virtual ~FlaggingVisitor() = default;
    _NO_COPY_OR_MOVE(FlaggingVisitor);

    /** The visitor method - flags the StateInfo with the flags. */
    virtual void visit(StateInfo *info) override;
    /** The flags that will be set by this visitor. */
    ChangeFlags flagsToSet_;
};

} /* namespace abt */

#endif /* SOLVER_FLAGGINGVISITOR_HPP_ */
