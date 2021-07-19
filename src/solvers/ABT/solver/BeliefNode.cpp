/** @file BeliefNode.cpp
 *
 * Contains the implementation for the BeliefNode class.
 */
#include "BeliefNode.hpp"

#include <map>                          // for _Rb_tree_iterator, map<>::iterator, map
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution
#include <set>
#include <tuple>                        // for tie, tuple
#include <utility>                      // for pair, make_pair, move

#include "oppt/global.hpp"                     // for RandomGenerator, make_unique

#include "cached_values.hpp"

#include "ActionNode.hpp"               // for ActionNode
#include "BeliefTree.hpp"
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "Solver.hpp"                   // for Solver

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/HistoricalData.hpp"
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"                    // for State

#include "belief-estimators/estimators.hpp"

#include "search/action-choosers/choosers.hpp"
#include "search/search_interface.hpp"

#include "mappings/actions/ActionMapping.hpp"
#include "mappings/actions/ActionPool.hpp"
#include "mappings/observations/ObservationMapping.hpp"
#include "mappings/observations/ObservationPool.hpp"

namespace abt
{
BeliefNode::BeliefNode(Solver* solver) :
    BeliefNode(-1, nullptr, solver)
{
}
BeliefNode::BeliefNode(ObservationMappingEntry* parentEntry, Solver* solver) :
    BeliefNode(-1, parentEntry, solver)
{
}

BeliefNode::BeliefNode(long id, ObservationMappingEntry* parentEntry, Solver* solver) :
    solver_(solver),
    id_(id),
    depth_(-1),
    parentEntry_(parentEntry),
    data_(nullptr),
    particles_(),
    nStartingSequences_(0),
    actionMap_(nullptr),
    cachedValues_(),
    valueEstimator_(nullptr)
{

    // Correctly calculate the depth based on the parent node.
    if (parentEntry_ == nullptr) {
        depth_ = 0;
    } else {
        depth_ = getParentBelief()->getDepth() + 1;
    }

    // Add this node to the index in the tree.
    solver_->getPolicy()->addNode(this);
}

// The destructor must remove the node from the solver's index.
BeliefNode::~BeliefNode()
{
    solver_->getPolicy()->removeNode(this);
}

void BeliefNode::clear() {

}

/* ----------------- Useful calculations ------------------- */
FloatType BeliefNode::distL1Independent(BeliefNode* b) const
{
    FloatType dist = 0.0;
    for (HistoryEntry * entry1 : particles_) {
        for (HistoryEntry * entry2 : b->particles_) {
            dist += entry1->getState()->distanceTo(*entry2->getState());
        }
    }
    FloatType averageDist = dist / (getNumberOfParticles() * b->getNumberOfParticles());
    if (averageDist < 0) {
        debug::show_message("ERROR: Distance < 0 between beliefs.");
    } else if (averageDist == 0) {
        debug::show_message("NOTE: Identical belief nodes found!");
    }
    return averageDist;
}

/* -------------------- Simple getters ---------------------- */
long BeliefNode::getId() const
{
    return id_;
}
long BeliefNode::getDepth() const
{
    return depth_;
}
long BeliefNode::getNumberOfParticles() const
{
    return particles_.size();
}
long BeliefNode::getNumberOfStartingSequences() const
{
    return nStartingSequences_;
}
std::vector<State const*> BeliefNode::getStates() const
{
    std::vector<State const*> states;
    for (HistoryEntry * entry : particles_) {
        if (!entry) {
            debug::show_message("WARNING: history entry in particles is null");
        } else {
            states.push_back(entry->getState());
        }
    }
    return states;
}

std::vector<State const*> BeliefNode::getNonTerminalStates() const {
    std::vector<State const*> states;
    for (HistoryEntry * entry : particles_) {
        if (entry && entry->getStateInfo() && entry->getStateInfo()->isTerminal() == false) {
            auto state = entry->getState();
            if (state)
                states.push_back(state);
        }
    }

    return states;
}

/* -------------------- Tree-related getters  ---------------------- */
ActionMapping* BeliefNode::getMapping() const
{
    return actionMap_.get();
}
ObservationMappingEntry* BeliefNode::getParentEntry() const
{
    return parentEntry_;
}
ActionNode* BeliefNode::getParentActionNode() const
{
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return parentEntry_->getMapping()->getOwner();
}
HistoricalData* BeliefNode::getHistoricalData() const
{
    return data_.get();
}
BeliefNode* BeliefNode::getParentBelief() const
{
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return getParentActionNode()->getParentEntry()->getMapping()->getOwner();
}
std::unique_ptr<Observation> BeliefNode::getLastObservation() const
{
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return parentEntry_->getObservation();
}
std::unique_ptr<Action> BeliefNode::getLastAction() const
{
    if (parentEntry_ == nullptr) {
        return nullptr;
    }
    return getParentActionNode()->getParentEntry()->getAction();
}
BeliefNode* BeliefNode::getChild(Action const& action, Observation const& obs) const
{
    ActionNode* node = actionMap_->getActionNode(action);
    if (node == nullptr) {
        return nullptr;
    }
    return node->getChild(obs);
}

/* ----------------- Management of cached values ------------------- */
BaseCachedValue* BeliefNode::addCachedValue(std::unique_ptr<BaseCachedValue> value)
{
    BaseCachedValue* rawPtr = value.get();
    cachedValues_[rawPtr] = std::move(value);
    return rawPtr;
}
void BeliefNode::removeCachedValue(BaseCachedValue* value)
{
    cachedValues_.erase(value);
}

/* ------------ Value estimation and action selection -------------- */
void BeliefNode::setValueEstimator(CachedValue<FloatType>* estimator)
{
    valueEstimator_ = estimator;
}
std::unique_ptr<Action> BeliefNode::getRecommendedAction() const
{
    return solver_->getRecommendationStrategy()->getAction(this);
}
FloatType BeliefNode::getCachedValue() const
{
    return valueEstimator_->getCache();
}
void BeliefNode::recalculateValue()
{
    valueEstimator_->updateCache();
}

/* -------------------- Core tree-related methods  ---------------------- */
BeliefNode* BeliefNode::createOrGetChild(Action const& action,
        Observation const& obs)
{
    ActionNode* actionNode = actionMap_->getActionNode(action);
    if (actionNode == nullptr) {
        actionNode = actionMap_->createActionNode(action);
        actionNode->setMapping(solver_->getObservationPool()->createObservationMapping(actionNode));
    }
    BeliefNode* childNode;
    bool isNew;
    std::tie(childNode, isNew) = actionNode->createOrGetChild(solver_, obs);
    if (isNew) {
        // If we've created a new node, we also need to create some of its key elements.
        if (data_ != nullptr) {
            childNode->setHistoricalData(data_->createChild(action, obs));
        }
        childNode->setMapping(solver_->getActionPool()->createActionMapping(childNode));
        solver_->getEstimationStrategy()->setValueEstimator(solver_, childNode);
    }

    return childNode;
}

/* ============================ PRIVATE ============================ */

/* -------------- Particle management / sampling ---------------- */
void BeliefNode::addParticle(HistoryEntry* newHistEntry)
{
    particles_.add(newHistEntry);
    if (newHistEntry->getId() == 0) {
        nStartingSequences_++;
    }
}

void BeliefNode::removeParticle(HistoryEntry* histEntry)
{
    particles_.remove(histEntry);
    if (histEntry->getId() == 0) {
        nStartingSequences_--;
    }
}

/* -------------------- Tree-related setters  ---------------------- */
void BeliefNode::setMapping(std::unique_ptr<ActionMapping> mapping)
{
    actionMap_ = std::move(mapping);
}
void BeliefNode::setHistoricalData(std::unique_ptr<HistoricalData> data)
{
    data_ = std::move(data);
}
} /* namespace abt */
