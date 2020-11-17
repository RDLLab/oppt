/** @file SpatialIndexVisitor.cpp
 *
 * Contains the implementation of SpatialIndexVisitor.
 */
#include "SpatialIndexVisitor.hpp"

#include <vector>

#include <spatialindex/SpatialIndex.h>

#include "solvers/ABT/solver/StatePool.hpp"

namespace abt {

SpatialIndexVisitor::SpatialIndexVisitor(StatePool *statePool) :
        statePool_(statePool) {
}

SpatialIndexVisitor::~SpatialIndexVisitor() {
}

void SpatialIndexVisitor::visitNode(const SpatialIndex::INode &/*node*/) {
}

/** This is the key visit method we need to use. Since we use the ID of the StateInfo as the ID
 * of the data in the SpatialIndex, we can look up the StateInfo by ID and pass it to the
 * visit() method.
 */
void SpatialIndexVisitor::visitData(const SpatialIndex::IData &data) {
        visit(statePool_->getInfoById(data.getIdentifier()));
}

void SpatialIndexVisitor::visitData(std::vector<const SpatialIndex::IData*> &/*v*/) {
}

StatePool *SpatialIndexVisitor::getStatePool() const {
    return statePool_;
}

} /* namespace abt */
