/** @file FlaggingVisitor.cpp
 *
 * Contains the implementation of the FlaggingVisitor class.
 */
#include "FlaggingVisitor.hpp"

#include "solvers/ABT/solver/changes/ChangeFlags.hpp"
#include "solvers/ABT/solver/StatePool.hpp"

namespace abt {

FlaggingVisitor::FlaggingVisitor(StatePool *pool,
        ChangeFlags flagsToSet) :
                SpatialIndexVisitor(pool),
                flagsToSet_(flagsToSet) {
}

void FlaggingVisitor::visit(StateInfo* info) {
    getStatePool()->setChangeFlags(info, flagsToSet_);
}

} /* namespace abt */
