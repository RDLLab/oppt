/** @file indexing/RTree.cpp
 *
 * Contains the implementation of the RTree class, which is a thin wrapper for the libspatialindex
 * ISpatialIndex interface that also implements the ABT interface StateIndex.
 */
#include "RTree.hpp"

#include <memory>

#include <spatialindex/SpatialIndex.h>
#include <spatialindex/RTree.h>
#include <spatialindex/tools/Tools.h>

#include "oppt/global.hpp"

#include "solvers/ABT/solver/StateInfo.hpp"

#include "solvers/ABT/solver/abstract-problem/VectorState.hpp"
#include "SpatialIndexVisitor.hpp"

using std::cout;
using std::endl;

namespace abt {
RTree::RTree(unsigned int nSDim) :
        StateIndex(),
        nSDim_(nSDim),
        properties_(nullptr),
        storageManager_(nullptr),
        tree_(nullptr) {	    
    reset();
}

void RTree::reset() {
    // First we set up the properties for the RTree correctly.
    properties_ = std::make_unique<Tools::PropertySet>();
    Tools::Variant var;

    // We use an R*-tree since this should be OK for general-purpose use.
    var.m_varType = Tools::VT_LONG;
    var.m_val.lVal = SpatialIndex::RTree::RV_RSTAR;
    properties_->setProperty("TreeVariant", var);

    // We set it to have the correct # of dimensions.
    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = nSDim_;
    properties_->setProperty("Dimension", var);


    // The IndexCapacity, LeafCapacity and FillFactor are chosen as defaults;
    // it may be useful to be able to tune them for performance.
    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties_->setProperty("IndexCapacity", var);

    var.m_varType = Tools::VT_ULONG;
    var.m_val.ulVal = 100;
    properties_->setProperty("LeafCapacity", var);

    var.m_varType = Tools::VT_DOUBLE;
    var.m_val.dblVal = 0.7;
    properties_->setProperty("FillFactor", var);

    // Now we create a new StorageManager and a new tree.
    tree_.reset(nullptr);
    storageManager_.reset(
            SpatialIndex::StorageManager::returnMemoryStorageManager(*properties_));
    tree_.reset(SpatialIndex::RTree::returnRTree(*storageManager_, *properties_));
}

void RTree::addStateInfo(StateInfo *stateInfo) {
    // The ID of the StateInfo will be stored as the ID in the RTree
    SpatialIndex::id_type stateId = stateInfo->getId();

    // Now we convert the state to a SpatialIndex::Point for storage in the RTree.
    std::vector<FloatType> vectorData = static_cast<VectorState const *>(
            stateInfo->getState())->asVector();
    SpatialIndex::Point point(&vectorData[0], nSDim_);

    // We're not storing any data at all - the ID is enough.
    tree_->insertData(0, nullptr, point, stateId);
}

void RTree::removeStateInfo(StateInfo *stateInfo) {
    // The ID of the StateInfo will be stored as the ID in the RTree
    SpatialIndex::id_type stateId = stateInfo->getId();

    // Now we convert the state to a SpatialIndex::Point
    std::vector<FloatType> vectorData = static_cast<VectorState const *>(
            stateInfo->getState())->asVector();
    SpatialIndex::Point point(&vectorData[0], nSDim_);

    // Delete the RTree entry.
    tree_->deleteData(point, stateId);
}

void RTree::boxQuery(SpatialIndexVisitor& visitor,
        std::vector<FloatType> lowCorner, std::vector<FloatType> highCorner) {
    // Turn the corners into a SpatialIndex::Region
    SpatialIndex::Region region(&lowCorner[0], &highCorner[0], nSDim_);
    // Now perform the query
    tree_->containsWhatQuery(region, visitor);
}

} /* namespace abt */
