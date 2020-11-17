/** @file ActionNode.cpp
 *
 * Contains the implementation of ActionNode.
 */
#include "ActionNode.hpp"

#include <memory>                       // for unique_ptr
#include <utility>                      // for make_pair, move, pair
#include <vector>                       // for vector

#include "BeliefNode.hpp"
#include "Solver.hpp"

#include "belief-estimators/estimators.hpp"

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Observation.hpp"              // for Observation

#include "mappings/actions/ActionPool.hpp"       // for ActionPool
#include "mappings/observations/ObservationMapping.hpp"       // for ObservationMapping

namespace abt {
ActionNode::ActionNode() :
        parentEntry_(nullptr),
        observationMap_(nullptr) {
}

ActionNode::ActionNode(ActionMappingEntry *parentEntry) :
        parentEntry_(parentEntry),
        observationMap_(nullptr) {
}

// Default destructor
ActionNode::~ActionNode() {
}

/* -------------------- Tree-related getters  ---------------------- */
ObservationMapping *ActionNode::getMapping() const {
    return observationMap_.get();
}
ActionMappingEntry *ActionNode::getParentEntry() const {
    return parentEntry_;
}
BeliefNode *ActionNode::getParentBelief() const {
    return parentEntry_->getMapping()->getOwner();
}
BeliefNode *ActionNode::getChild(Observation const &obs) const {
    return observationMap_->getBelief(obs);
}


/* ============================ PRIVATE ============================ */


/* -------------------- Tree-related setters  ---------------------- */
void ActionNode::setMapping(std::unique_ptr<ObservationMapping> mapping) {
    observationMap_ = std::move(mapping);
}

/* -------------------- Tree-related methods  ---------------------- */
std::pair<BeliefNode *, bool> ActionNode::createOrGetChild(Solver */*solver*/,
        Observation const &obs) {
    BeliefNode *childNode = getChild(obs);
    bool added = false;
    if (childNode == nullptr) {
        childNode = observationMap_->createBelief(obs);
        added = true;
    }
    return std::make_pair(childNode, added);
}
} /* namespace abt */
