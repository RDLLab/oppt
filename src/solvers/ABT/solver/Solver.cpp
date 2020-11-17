/** @file Solver.cpp
 *
 * Contains the implementation of the Solver class.
 */
#include "Solver.hpp"

#include <cmath>                        // for pow, exp
#include <cstdio>

#include <algorithm>                    // for max
#include <iostream>                     // for operator<<, ostream, basic_ostream, endl, basic_ostream<>::__ostream_type, cout
#include <limits>
#include <memory>                       // for unique_ptr
#include <random>                       // for uniform_int_distribution, bernoulli_distribution
#include <set>                          // for set, _Rb_tree_const_iterator, set<>::iterator
#include <tuple>                        // for tie, tuple
#include <type_traits>                  // for remove_reference<>::type
#include <utility>                      // for move, make_pair, pair
#include <vector>                       // for vector, vector<>::iterator, vector<>::reverse_iterator

#include "oppt/global.hpp"                     // for RandomGenerator

#include "abstract-problem/Action.hpp"                   // for Action
#include "abstract-problem/Model.hpp"                    // for Model::StepResult, Model
#include "abstract-problem/ModelChange.hpp"                    // for Model::StepResult, Model
#include "abstract-problem/Observation.hpp"              // for Observation
#include "abstract-problem/State.hpp"                    // for State, operator<<

#include "belief-estimators/estimators.hpp"

#include "changes/ChangeFlags.hpp"               // for ChangeFlags, ChangeFlags::UNCHANGED, ChangeFlags::ADDOBSERVATION, ChangeFlags::ADDBODY, ChangeFlags::ADDSTATE, ChangeFlags::DELSTATE, ChangeFlags::REWARD, ChangeFlags::TRANSITION
#include "changes/HistoryCorrector.hpp"

#include "mappings/actions/ActionMapping.hpp"
#include "mappings/actions/ActionPool.hpp"
#include "mappings/observations/ObservationMapping.hpp"
#include "mappings/observations/ObservationPool.hpp"

#include "search/SearchStatus.hpp"
#include "search/search_interface.hpp"

#include "serialization/Serializer.hpp"               // for Serializer

#include "indexing/RTree.hpp"
#include "indexing/SpatialIndexVisitor.hpp"

#include "ActionNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefNode.hpp"               // for BeliefNode, BeliefNode::startTime
#include "BeliefTree.hpp"               // for BeliefTree
#include "Histories.hpp"                // for Histories
#include "HistoryEntry.hpp"             // for HistoryEntry
#include "HistorySequence.hpp"          // for HistorySequence
#include "StateInfo.hpp"                // for StateInfo
#include "StatePool.hpp"                // for StatePool
#include "solvers/ABT/ABTOptions.hpp"
#include "solvers/ABT/robotModel/solver/StateInfo.hpp"
#include <boost/timer.hpp>

using std::cout;
using std::endl;

namespace abt
{
Solver::Solver(std::unique_ptr<Model> model) :
    model_(std::move(model)),
    options_(model_->getOptions()),
    serializer_(model_->createSerializer(this)),
    actionPool_(nullptr),
    observationPool_(nullptr),
    statePool_(nullptr),
    histories_(nullptr),
    policy_(nullptr),
    historyCorrector_(nullptr),
    searchStrategy_(nullptr),
    recommendationStrategy_(nullptr),
    estimationStrategy_(nullptr),
    nodesToBackup_(),
    changeRoot_(nullptr),
    isAffectedMap_()
{
}

// Default destructor
Solver::~Solver()
{
}

/* ------------------ Simple getters. ------------------- */
BeliefTree* Solver::getPolicy() const
{
    return policy_.get();
}
StatePool* Solver::getStatePool() const
{
    return statePool_.get();
}
Model* Solver::getModel() const
{
    return model_.get();
}
oppt::Options const* Solver::getOptions() const
{
    return options_;
}
ActionPool* Solver::getActionPool() const
{
    return actionPool_.get();
}
ObservationPool* Solver::getObservationPool() const
{
    return observationPool_.get();
}
EstimationStrategy* Solver::getEstimationStrategy() const
{
    return estimationStrategy_.get();
}
SelectRecommendedActionStrategy* Solver::getRecommendationStrategy() const
{
    return recommendationStrategy_.get();
}
Serializer* Solver::getSerializer() const
{
    return serializer_.get();
}

/* ------------------ Initialization methods ------------------- */
void Solver::initializeEmpty()
{
    // Basic initialization.
    initialize();

    // Create new instances of these.
    actionPool_ = model_->createActionPool(this);
    observationPool_ = model_->createObservationPool(this);

    // Initialize the root node properly.
    policy_->initializeRoot();
}

/* ------------------- Policy mutators ------------------- */
long Solver::improvePolicy(BeliefNode* startNode, long numberOfHistories, long maximumDepth,
                           FloatType timeout)
{

    if (numberOfHistories < 0) {
        numberOfHistories = static_cast<oppt::ABTExtendedOptions const*>(options_)->historiesPerStep;
    }
    if (maximumDepth < 0) {
        maximumDepth = static_cast<oppt::ABTExtendedOptions const*>(options_)->maximumDepth;
        if (startNode != nullptr &&
                !static_cast<oppt::ABTExtendedOptions const*>(options_)->isAbsoluteHorizon) {
            maximumDepth += startNode->getDepth();
        }
    }
    if (timeout < 0) {
        timeout = static_cast<oppt::ABTExtendedOptions const*>(options_)->stepTimeout;
    }

    // No timeout => end at t=inf
    if (timeout == 0) {
        // 0 => no timeout (infinity)
        timeout = std::numeric_limits<FloatType>::infinity();
    }

    // Retrieve the sampling function to use.
    std::function<StateInfo *()> sampler = getStateSampler(startNode);
    if (sampler == nullptr) {
        return 0;
    }

    // Null start node => use the root.
    if (startNode == nullptr) {
        startNode = policy_->getRoot();
    }

    FloatType startTime = oppt::clock_ms();
    long actualNumHistories = multipleSearches(startNode,
                              sampler,
                              maximumDepth,
                              numberOfHistories,
                              startTime + timeout);
    FloatType totalTimeTaken = oppt::clock_ms() - startTime;
    if (options_->hasVerboseOutput) {
        cout << actualNumHistories << " histories in " << totalTimeTaken << "ms." << endl;
    }

    return actualNumHistories;
}



void Solver::resetTree(BeliefNode* newRoot)
{
    changeRoot_ = nullptr;
    isAffectedMap_.clear();
    nodesToBackup_.clear();

    std::vector<StateInfo*> allParticles;

    // Filter out only the valid particles for the new belief.
    for (HistoryEntry * entry : newRoot->particles_) {
        StateInfo* info = entry->stateInfo_;
        if (model_->isValid(*info->getState())) {
            allParticles.push_back(entry->stateInfo_);
        }
    }

    HistoricalData* oldData = newRoot->getHistoricalData();
    std::unique_ptr<HistoricalData> newData;
    if (oldData != nullptr) {
        newData = oldData->copy();
    }
    newRoot = policy_->reset();
    newRoot->data_ = std::move(newData);
    newRoot->setMapping(actionPool_->createActionMapping(newRoot));
    estimationStrategy_->setValueEstimator(this, newRoot);
    histories_->reset();

    // Clear the stored history entries for each StateInfo in the pool.
    for (std::unique_ptr<StateInfo> const & info : statePool_->statesByIndex_) {
        info->usedInHistoryEntries_.clear();
    }

    // Now fill the re-created belief node with the particles from the old one.
    for (StateInfo * info : allParticles) {
        // Create a new history sequence and entry for each new particle.
        HistorySequence* histSeq = histories_->createSequence();
        HistoryEntry* histEntry = histSeq->addEntry();
        histEntry->registerState(info);
        histEntry->registerNode(newRoot);
    }
}

long Solver::pruneSiblings(BeliefNode* node)
{
    ObservationMappingEntry* entry = node->getParentEntry();
    if (entry == nullptr) {
        return 0;
    }

    long nSequencesDeleted = 0;

    // Prune siblings that share an action, but not an observation.
    ObservationMapping* obsMap = entry->getMapping();
    for (ObservationMappingEntry const * sibling : obsMap->getChildEntries()) {
        if (sibling != entry) {
            nSequencesDeleted += pruneSubtree(sibling->getBeliefNode());
        }
    }

    ActionMappingEntry* actionEntry = obsMap->getOwner()->getParentEntry();
    ActionMapping* actionMapping = actionEntry->getMapping();
    for (ActionMappingEntry const * actionSibling : actionEntry->getMapping()->getChildEntries()) {
        if (actionSibling != actionEntry) {
            ObservationMapping* siblingObsMap = actionSibling->getActionNode()->getMapping();
            for (ObservationMappingEntry const * obsSibling : siblingObsMap->getChildEntries()) {
                nSequencesDeleted += pruneSubtree(obsSibling->getBeliefNode());
            }
            // Now delete the action mapping entry.
            actionMapping->deleteChild(actionSibling);
        }
    }

    // Backup the parent node so the value estimate remains OK.
    doBackup();

    return nSequencesDeleted;
}

long Solver::pruneSubtree(BeliefNode* root)
{
    // Delete all history sequences going into this subtree.
    long nSequencesDeleted = 0;
    for (HistoryEntry * entry : root->particles_) {
        histories_->deleteSequence(entry->owningSequence_);
        nSequencesDeleted++;
    }

    ObservationMappingEntry* entry = root->getParentEntry();
    if (entry != nullptr) {
        addNodeToBackup(root->getParentBelief());
        entry->getMapping()->deleteChild(entry);
    }

    return nSequencesDeleted;
}

/* ------------------- Change handling methods ------------------- */
BeliefNode* Solver::getChangeRoot() const
{
    return changeRoot_;
}

void Solver::setChangeRoot(BeliefNode* changeRoot)
{
    changeRoot_ = changeRoot;
}

bool Solver::isAffected(BeliefNode const* node)
{
    if (changeRoot_ == nullptr) {
        // No change root => all nodes are affected.
        return true;
    }
    if (node == changeRoot_) {
        // Same node => must be affected.
        return true;
    }

    long rootDepth = changeRoot_->getDepth();
    if (node->getDepth() <= rootDepth) {
        // Not below the root and not the root => can't be affected.
        return false;
    }

    // If it's deeper, look it up in the mapping.
    std::unordered_map<BeliefNode const*, bool>::iterator it = isAffectedMap_.find(node);
    if (it != isAffectedMap_.end()) {
        return it->second;
    }

    // We have to traverse the tree to see if it's a descendant.
    std::vector<BeliefNode const*> ancestors;
    bool isDescendedFromChangeRoot = false;
    while (true) {
        ancestors.push_back(node);
        node = node->getParentBelief();
        // If we hit the root, we must be descended from it.
        if (node == changeRoot_) {
            isDescendedFromChangeRoot = true;
            break;
        }
        // If we reach the same depth without hitting the root, we aren't descended from it.
        if (node->getDepth() == rootDepth) {
            break;
        }

        it = isAffectedMap_.find(node);
        // If we hit a node for which we know whether it's descended, we copy its state.
        if (it != isAffectedMap_.end()) {
            isDescendedFromChangeRoot = it->second;
            break;
        }
    }

    // Add ancestors to the mapping for efficient lookup later on.
    for (BeliefNode const * ancestor : ancestors) {
        isAffectedMap_[ancestor] = isDescendedFromChangeRoot;
    }

    return isDescendedFromChangeRoot;
}

void Solver::applyChanges()
{
    std::unordered_set<HistorySequence*> affectedSequences;
    for (StateInfo * stateInfo : statePool_->getAffectedStates()) {
        if (changes::has_flags(stateInfo->changeFlags_, ChangeFlags::DELETED)) {
            // Deletion implies the prior transition must be invalid.
            stateInfo->changeFlags_ |= ChangeFlags::TRANSITION_BEFORE;
        }

        for (HistoryEntry * entry : stateInfo->usedInHistoryEntries_) {
            BeliefNode* node = entry->getAssociatedBeliefNode();

            // If this node isn't affected, ignore it.
            if (!isAffected(node)) {
                continue;
            }

            HistorySequence* sequence = entry->owningSequence_;
            HistoryEntry::IdType entryId = entry->entryId_;

            // True iff any changes have been applied to this sequence.
            bool hasChanged = false;
            ChangeFlags allFlags = stateInfo->changeFlags_;

            if (node == changeRoot_) {
                // Any flags applying to previous states must be ignored.
                allFlags &= ~ChangeFlags::TRANSITION_BEFORE;
                allFlags &= ~ChangeFlags::OBSERVATION_BEFORE;
                allFlags &= ~ChangeFlags::HEURISTIC;

                // If a state at the change root is deleted, just delete the whole sequence.
                if (changes::has_flags(allFlags, ChangeFlags::DELETED)) {
                    hasChanged = true;
                    sequence->setChangeFlags(0, ChangeFlags::DELETED);
                }
            }

            // TRANSITION_BEFORE => TRANSITION for previous entry.
            if (changes::has_flags(allFlags, ChangeFlags::TRANSITION_BEFORE)) {
                allFlags &= ~ChangeFlags::TRANSITION_BEFORE;
                if (entryId > 0) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::TRANSITION);
                }
            }

            // OBSERVATION_BEFORE => OBSERVATION for previous entry.
            if (changes::has_flags(allFlags, ChangeFlags::OBSERVATION_BEFORE)) {
                allFlags &= ~ChangeFlags::OBSERVATION_BEFORE;
                if (entryId > 0) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId - 1, ChangeFlags::OBSERVATION);
                }
            }

            // Heuristic changes only apply if it's the last entry in the sequence.
            if (changes::has_flags(allFlags, ChangeFlags::HEURISTIC)) {
                allFlags &= ~ChangeFlags::HEURISTIC;
                if (entry->action_ == nullptr) {
                    hasChanged = true;
                    sequence->setChangeFlags(entryId, ChangeFlags::HEURISTIC);
                }
            }

            // Apply any remaining flags, if any.
            if (allFlags != ChangeFlags::UNCHANGED) {
                hasChanged = true;
                sequence->setChangeFlags(entryId, allFlags);
            }
            if (hasChanged) {
                // The sequence is only affected if one of the changes went through.
                affectedSequences.insert(sequence);
            }
        }
    }

    long numAffected = affectedSequences.size();

    if (options_->hasVerboseOutput) {
        cout << "Must revise " << numAffected << " of ";
        cout << histories_->getNumberOfSequences() << " histories!" << endl;
    }

    // Delete and remove any sequences where the first state has been deleted.
    std::unordered_set<HistorySequence*>::iterator it = affectedSequences.begin();
    while (it != affectedSequences.end()) {
        HistorySequence* sequence = *it;
        if (changes::has_flags(sequence->getFirstEntry()->changeFlags_, ChangeFlags::DELETED)) {
            it = affectedSequences.erase(it);
            // Now we undo the sequence, and delete it entirely.
            updateSequence(sequence, -1);
            histories_->deleteSequence(sequence);
        } else {
            it++;
        }
    }

    if (options_->hasVerboseOutput) {
        cout << "Deleted " << numAffected - affectedSequences.size() << " histories!" << endl;
    }

    // Revise all of the histories.
    historyCorrector_->reviseHistories(affectedSequences);

    if (options_->hasVerboseOutput) {
        cout << "Revision complete. Backing up..." << endl;
    }

    // Finish the deferred backup so that solvers requiring backed-up values will work properly.
    doBackup();

    // Reset the set of affected states in the pool.
    statePool_->resetAffectedStates();

    // Now we extend sequences with illegal actions.
    if (options_->hasVerboseOutput) {
        cout << "Using search on " << affectedSequences.size() << " histories!" << endl;
    }

    // Extend and backup each sequence.
    for (HistorySequence * sequence : affectedSequences) {
        long maximumDepth =
            static_cast<oppt::ABTExtendedOptions const*>(options_)->maximumDepth;
        if (static_cast<oppt::ABTExtendedOptions const*>(options_)->isAbsoluteHorizon) {
            maximumDepth += sequence->getFirstEntry()->getAssociatedBeliefNode()->getDepth();
        }
        searchStrategy_->extendAndBackup(sequence, maximumDepth);
    }

    // Backup all the way to the root to keep the tree consistent.
    doBackup();

    // Clear the map of affected nodes, to make sure it doesn't keep nodes that may be deleted.
    isAffectedMap_.clear();
}

/* ------------------ Display methods  ------------------- */
void Solver::printBelief(BeliefNode* belief, std::ostream& os)
{
    os << belief->getCachedValue();
    os << " from " << belief->getNumberOfParticles() << " p.";
    os << " with " << belief->getNumberOfStartingSequences() << " starts.";
    os << endl;
    os << "Action children: " << endl;
    typedef std::pair<abt::ActionMappingEntry const*, std::string> EntryWithMarker;
    std::multimap<FloatType, EntryWithMarker> actionValues;
    size_t longestMarker = 0;
    for (abt::ActionMappingEntry const * entry : belief->getMapping()->getVisitedEntries()) {
        EntryWithMarker entryWithMarker(entry, entry->getMarker());
        longestMarker = std::max(longestMarker, entryWithMarker.second.length());
        actionValues.insert(std::make_pair(entry->getMeanQValue(), std::move(entryWithMarker)));
    }
    for (auto it = actionValues.rbegin(); it != actionValues.rend(); it++) {
        oppt::print_double(it->first, os, 8, 2, std::ios_base::fixed | std::ios_base::showpos);
        os << ": ";
        oppt::print_with_width(it->second.second, os, longestMarker);
        std::ostringstream sstr;
        sstr << *it->second.first->getAction();
        oppt::print_with_width(sstr.str(), os, 17);
        oppt::print_with_width(it->second.first->getVisitCount(), os, 8);
        os << endl;
    }
}

void Solver::printTree(std::ostream& /*os*/)
{
}

/* -------------- Management of deferred backpropagation. --------------- */
bool Solver::isBackedUp() const
{
    return nodesToBackup_.empty();
}

void Solver::doBackup()
{
    while (!nodesToBackup_.empty()) {
        auto firstEntry = nodesToBackup_.cbegin();
        long depth = firstEntry->first;
        for (BeliefNode * node : firstEntry->second) {
            if (depth == 0) {
                node->recalculateValue();
            } else {
                FloatType oldQValue = node->getCachedValue();
                node->recalculateValue();
                FloatType deltaQValue = node->getCachedValue() - oldQValue;
                long nContinuations = node->getMapping()->getTotalVisitCount()
                                      - node->getNumberOfStartingSequences();
                FloatType deltaTotalQ =
                    static_cast<oppt::ABTExtendedOptions const*>(options_)->discountFactor * nContinuations * deltaQValue;

                ActionMappingEntry* parentActionEntry =
                    node->getParentActionNode()->getParentEntry();
                if (parentActionEntry->update(0, deltaTotalQ)) {
                    addNodeToBackup(parentActionEntry->getMapping()->getOwner());
                }
            }
        }
        nodesToBackup_.erase(firstEntry);
    }
}


/* ------------------ Methods to update the q-values in the tree. ------------------- */
void Solver::updateSequence(HistorySequence* sequence, int sgn, long firstEntryId,
                            bool propagateQChanges)
{
    // Cannot update sequences of length <= 1.
    if (sequence->getLength() <= 1) {
        return;
    }

    FloatType discountFactor = static_cast<oppt::ABTExtendedOptions const*>(options_)->discountFactor;


    // Traverse the sequence in reverse.
    auto it = sequence->entrySequence_.crbegin();

    // The last entry is used only for the heuristic estimate.
    FloatType deltaTotalQ = (*it)->immediateReward_;
    it++;
    BeliefNode* node;
    while (true) {
        // Apply discount and add the immediate reward.
        deltaTotalQ = deltaTotalQ * discountFactor + (*it)->immediateReward_;
        node = (*it)->getAssociatedBeliefNode();
        ActionMapping* mapping = node->getMapping();
        ActionMappingEntry* entry = mapping->getEntry(*(*it)->getAction());
        // Update the action value and visit count.        
        entry->update(sgn, sgn * deltaTotalQ);

        // Update the observation visit count.
        const Observation* obs = (*it)->getObservation();

        ObservationMappingEntry* obsEntry = (
                                                entry->getActionNode()->getMapping()->getEntry(*(*it)->getObservation()));

        if (!obsEntry)
            cout << "NO OBS ENTRY" << endl;
        obsEntry->updateVisitCount(sgn);

        // If we've gone past the source node, we don't need to update further.
        // Backpropagation may need to go further, but we can simply defer it.
        it++;
        if (it == sequence->entrySequence_.crend() || (*it)->entryId_ < firstEntryId) {
            addNodeToBackup(node);
            break;
        }

        FloatType oldQ = node->getCachedValue();
        deltaTotalQ = oldQ;
        if (propagateQChanges) {
            // Backpropagate the change in the q-value of the node.
            node->recalculateValue();
            deltaTotalQ = node->getCachedValue();
            //long nContinuations = mapping->getTotalVisitCount() - node->getNumberOfStartingSequences();
            //deltaTotalQ += (newQ - oldQ) * nContinuations;
        } else {
            // If we haven't backpropagated the change, we need to do it later.
            addNodeToBackup(node);
        }
    }
}

void Solver::updateEstimate(BeliefNode* node, FloatType deltaTotalQ, long deltaNContinuations)
{
    if (node->getParentEntry() == nullptr) {
        return;
    }

    deltaTotalQ += deltaNContinuations * node->getCachedValue();

    // Apply the discount factor.
    deltaTotalQ *= static_cast<oppt::ABTExtendedOptions const*>(options_)->discountFactor;

    ActionMappingEntry* parentActionEntry = node->getParentActionNode()->getParentEntry();
    if (parentActionEntry->update(0, deltaTotalQ)) {
        addNodeToBackup(parentActionEntry->getMapping()->getOwner());
    }
}

void Solver::updateImmediate(BeliefNode* node, Action const& action, Observation const& observation,
                             FloatType deltaTotalQ, long deltaNVisits)
{

    // all zero => no update required.
    if (deltaTotalQ == 0 && deltaNVisits == 0) {
        return;
    }

    // Retrieve the associated ActionMappingEntry.
    ActionMappingEntry* entry = node->getMapping()->getEntry(action);

    // Update the visit count for the observation.
    entry->getActionNode()->getMapping()->getEntry(observation)->updateVisitCount(deltaNVisits);

    // Update the action.
    if (entry->update(deltaNVisits, deltaTotalQ)) {
        addNodeToBackup(node);
    }
}


/* ============================ PRIVATE ============================ */

/* ------------------ Initialization methods ------------------- */
void Solver::initialize()
{
    // Core data structures
    statePool_ = std::make_unique<StatePool>(model_->createStateIndex());
    histories_ = std::make_unique<Histories>();
    policy_ = std::make_unique<BeliefTree>(this);
    policy_->reset();

    // Serializable model-specific customizations
    actionPool_ = nullptr;
    observationPool_ = nullptr;

    // Possible model-specific customizations
    historyCorrector_ = model_->createHistoryCorrector(this);
    searchStrategy_ = model_->createSearchStrategy(this);
    recommendationStrategy_ = model_->createRecommendationSelectionStrategy(this);
    estimationStrategy_ = model_->createEstimationStrategy(this);
}

/* ------------------ Episode sampling methods ------------------- */
std::function<StateInfo *()> Solver::getStateSampler(BeliefNode* node)
{
    // Nullptr => sample initial sates from the model.
    if (node == nullptr || node->getId() == 0) {
        return [this]() {
            return statePool_->createOrGetInfo(std::move(model_->sampleAnInitState()));
        };
    }

    // Filter out any terminal states.
    std::vector<StateInfo*> nonTerminalStates;
    for (long index = 0; index < node->getNumberOfParticles(); index++) {
        HistoryEntry* entry = node->particles_.get(index);
        if (!model_->isTerminal(*entry->getState())) {
            nonTerminalStates.push_back(entry->stateInfo_);
        }
    }

    // No non-terminal states => return an empty function.
    if (nonTerminalStates.empty()) {
        debug::show_message("ERROR: No non-terminal particles in the current node!");
        return std::function<StateInfo *()>();
    }

    RandomGenerator* randGen = model_->getRandomGenerator();
    return [&, nonTerminalStates]() {
        return model_->sampleParticle(nonTerminalStates);
    };
    /**return [randGen, nonTerminalStates]() {
        long index = std::uniform_int_distribution<long>(0, nonTerminalStates.size() - 1)(*randGen);
        return nonTerminalStates[index];
    };*/
}

long Solver::multipleSearches(BeliefNode* startNode, std::function<StateInfo *()> sampler,
                              long maximumDepth, long maxNumSearches, FloatType endTime)
{
    bool hasTimeout = true;
    if (endTime == std::numeric_limits<FloatType>::infinity()) {
        hasTimeout = false;
    }

    long numSearches = 0;
    while (true) {
        // If we've done enough searches, stop searching.
        if (maxNumSearches != 0 && numSearches >= maxNumSearches) {
            cout << "Max num searches reached. Time: " << oppt::clock_ms() << endl;
            cout << "End time: " << endTime << endl;
            break;
        }

        // If we've gone past the termination time, stop searching.
        if (maxNumSearches == 0 && hasTimeout && oppt::clock_ms() >= endTime) {
            break;
        }
        
        singleSearch(startNode, sampler(), maximumDepth);
        numSearches++;
    }

    // Backup all the way back to the root of the tree to maintain consistency.
    doBackup();

    return numSearches;
}

void Solver::singleSearch(BeliefNode* startNode, StateInfo* startStateInfo, long maximumDepth, Action *action)
{    
    HistorySequence* sequence = histories_->createSequence();
    HistoryEntry* firstEntry = sequence->addEntry();
    firstEntry->registerState(startStateInfo);
    if (startNode == getPolicy()->getRoot())
        firstEntry->registerNode(startNode);
    else
        firstEntry->registerNode(startNode, false);    
    continueSearch(sequence, maximumDepth, action);    
}

void Solver::continueSearch(HistorySequence* sequence, long maximumDepth, Action *action)
{
    // Extend and backup sequence.
    searchStrategy_->extendAndBackup(sequence, maximumDepth, action);
}

/* ------------------ Private deferred backup methods. ------------------- */
void Solver::addNodeToBackup(BeliefNode* node)
{
    nodesToBackup_[node->getDepth()].insert(node);
}

void Solver::removeNodeToBackup(BeliefNode* node)
{
    auto nodeSet = nodesToBackup_[node->getDepth()];
    nodeSet.erase(nodeSet.find(node));
}
} /* namespace abt */
