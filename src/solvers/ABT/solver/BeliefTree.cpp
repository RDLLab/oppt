/** @file BeliefTree.cpp
 *
 * Contains the implementation of BeliefTree.
 */
#include "BeliefTree.hpp"

#include <memory>                       // for unique_ptr
#include <vector>                       // for vector
#include <iostream>

#include "oppt/global.hpp"                     // for make_unique

#include "BeliefNode.hpp"               // for BeliefNode
#include "Solver.hpp"

#include "abstract-problem/Observation.hpp"

#include "belief-estimators/estimators.hpp"

#include "mappings/actions/ActionMapping.hpp"
#include "mappings/actions/ActionPool.hpp"

namespace abt {
BeliefTree::BeliefTree(Solver *solver) :
    solver_(solver),
    allNodes_(),
    root_(nullptr) {
}

// Do nothing!
BeliefTree::~BeliefTree() {
}

/* ------------------- Simple getters --------------------- */
BeliefNode *BeliefTree::getRoot() const {
    return root_.get();
}
BeliefNode *BeliefTree::getNode(long id) const {
    BeliefNode *node = allNodes_[id];    
    if (node->getId() != id) {
        std::ostringstream message;
        message << "ERROR: ID mismatch in Belief Tree - ID should be " << id;
        message << " but was " << node->getId();
        debug::show_message(message.str());;
    }
    return allNodes_[id];
}
long BeliefTree::getNumberOfNodes() const {
    return allNodes_.size();
}
std::vector<BeliefNode *> BeliefTree::getNodes() const {
    return allNodes_;
}

/* ============================ PRIVATE ============================ */


/* ------------------- Node index modification ------------------- */
void BeliefTree::addNode(BeliefNode *node) {
    long id = node->id_;
    if (id < 0) {
        // Negative ID => add it to the back of the vector.
        id = allNodes_.size();
        node->id_ = id;
        allNodes_.push_back(nullptr);
    }
    if (allNodes_[id] != nullptr) {
        debug::show_message("ERROR: Node already exists - overwriting!!");
    }
    allNodes_[id] = node;
}

void BeliefTree::removeNode(BeliefNode *node) {
    long id = node->id_;

    long lastNodeId = allNodes_.size() - 1;

    if (id < 0 || id > lastNodeId) {
        debug::show_message("ERROR: Node ID is out of bounds.");	
        return;
    }
    
    if (allNodes_[id] != node) {
        debug::show_message("ERROR: Node ID does not match index.");
        return;
    }

    // Now remove the node from the index.
    if (id < lastNodeId) {
        BeliefNode *lastNode = allNodes_[lastNodeId];
        lastNode->id_ = id;
        allNodes_[id] = lastNode;
    }
    allNodes_.pop_back();
}

/* ------------------- Tree modification ------------------- */
BeliefNode *BeliefTree::reset() {
    root_ = nullptr;
    root_ = std::make_unique<BeliefNode>(solver_);
    BeliefNode *rootPtr = root_.get();    
    return rootPtr;
}

void BeliefTree::initializeRoot() {
    root_->setHistoricalData(solver_->getModel()->createRootHistoricalData());
    root_->setMapping(solver_->getActionPool()->createActionMapping(root_.get()));
    solver_->getEstimationStrategy()->setValueEstimator(solver_, root_.get());
}
} /* namespace abt */
