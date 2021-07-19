/** @file discretized_actions.cpp
 *
 * Contains the implementations of the classes for discretized action mappings.
 */
#include "discretized_actions.hpp"

#include <algorithm>
#include <cmath>

#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"
#include "solvers/ABT/solver/abstract-problem/DiscretizedPoint.hpp"

#include "ActionMapping.hpp"
#include "ActionMappingEntry.hpp"
#include "ActionPool.hpp"

namespace abt {
/* ---------------------- DiscretizedActionPool ---------------------- */
std::unique_ptr<ActionMapping> DiscretizedActionPool::createActionMapping(BeliefNode *node) {
    return std::make_unique<DiscretizedActionMap>(node, this, createBinSequence(node));
}

/* ---------------------- DiscretizedActionMap ---------------------- */
DiscretizedActionMap::DiscretizedActionMap(BeliefNode *owner, DiscretizedActionPool *pool,
        std::vector<long> binSequence) :
        ActionMapping(owner),
                pool_(pool),
                numberOfBins_(pool_->getNumberOfBins()),
                entries_(std::make_unique<DiscretizedActionMapEntry[]>(numberOfBins_)),
                nChildren_(0),
                numberOfVisitedEntries_(0),
                binSequence_(binSequence.begin(), binSequence.end()),
                totalVisitCount_(0) {
    for (int i = 0; i < numberOfBins_; i++) {
        DiscretizedActionMapEntry &entry = entries_[i];
        entry.binNumber_ = i;
        entry.map_ = this;
        entry.isLegal_ = false;
    }

    // Only entries in the sequence are legal.
    for (long binNumber : binSequence_) {
        entries_[binNumber].isLegal_ = true;
    }
}

// Default destructor.
DiscretizedActionMap::~DiscretizedActionMap() {
}

ActionNode* DiscretizedActionMap::getActionNode(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return entries_[code].getActionNode();
}
ActionNode* DiscretizedActionMap::createActionNode(Action const &action) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    DiscretizedActionMapEntry &entry = entries_[code];

    std::unique_ptr<ActionNode> actionNode = std::make_unique<ActionNode>(&entry);
    ActionNode *node = actionNode.get();
    entry.childNode_ = std::move(actionNode);

    nChildren_++;
    return node;
}
long DiscretizedActionMap::getNChildren() const {
    return nChildren_;
}

void DiscretizedActionMap::deleteChild(ActionMappingEntry const *entry) {
    DiscretizedActionMapEntry &discEntry = const_cast<DiscretizedActionMapEntry &>(
                static_cast<DiscretizedActionMapEntry const &>(*entry));

    // Perform a negative update on the entry.
    discEntry.update(-discEntry.visitCount_, -discEntry.totalQValue_);

    // Now delete the child node.
    discEntry.childNode_ = nullptr;
}


std::vector<ActionMappingEntry const *> DiscretizedActionMap::getChildEntries() const  {
    std::vector<ActionMappingEntry const *> returnEntries;
    for (int i = 0; i < numberOfBins_; i++) {
        DiscretizedActionMapEntry const &entry = entries_[i];
        if (entry.childNode_ != nullptr) {
            returnEntries.push_back(&entry);
        }
    }
    return returnEntries;
}
long DiscretizedActionMap::getNumberOfVisitedEntries() const {
    return numberOfVisitedEntries_;
}
std::vector<ActionMappingEntry const *> DiscretizedActionMap::getVisitedEntries() const {
    std::vector<ActionMappingEntry const *> returnEntries;
    for (int i = 0; i < numberOfBins_; i++) {
        DiscretizedActionMapEntry const &entry = entries_[i];
        if (entry.visitCount_ > 0) {
            if (!entry.isLegal_) {
                debug::show_message("WARNING: Illegal entry with nonzero visit count!");
            }
            returnEntries.push_back(&entry);
        }
    }
    return returnEntries;
}
ActionMappingEntry *DiscretizedActionMap::getEntry(Action const &action) {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return &entries_[code];
}
ActionMappingEntry const *DiscretizedActionMap::getEntry(Action const &action) const {
    long code = static_cast<DiscretizedPoint const &>(action).getBinNumber();
    return &entries_[code];
}


std::unique_ptr<Action> DiscretizedActionMap::getNextActionToTry() {
    // No more bins to try -> no action to try.
    if (binSequence_.size() == 0) {
        return nullptr;
    }
    // Otherwise we sample a new action using the first bin to be tried.
    return pool_->sampleAnAction(binSequence_.getFirst());
}

long DiscretizedActionMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

/* ------------------- DiscretizedActionMapEntry ------------------- */
ActionMapping *DiscretizedActionMapEntry::getMapping() const {
    return map_;
}
std::unique_ptr<Action> DiscretizedActionMapEntry::getAction() const {
    return map_->pool_->sampleAnAction(binNumber_);
}
ActionNode *DiscretizedActionMapEntry::getActionNode() const {
    return childNode_.get();
}
long DiscretizedActionMapEntry::getVisitCount() const {
    return visitCount_;
}
FloatType DiscretizedActionMapEntry::getTotalQValue() const {
    return totalQValue_;
}
FloatType DiscretizedActionMapEntry::getMeanQValue() const {
    return meanQValue_;
}
bool DiscretizedActionMapEntry::isLegal() const {
    return isLegal_;
}


long DiscretizedActionMapEntry::getBinNumber() const {
    return binNumber_;
}


bool DiscretizedActionMapEntry::update(long deltaNVisits, FloatType deltaTotalQ) {
    if (deltaNVisits == 0 && deltaTotalQ == 0) {
        return false;
    }

    if (!std::isfinite(deltaTotalQ)) {
        debug::show_message("ERROR: Non-finite delta value!");        
    }

    if (deltaNVisits > 0 && !isLegal_) {
        debug::show_message("ERROR: Visiting an illegal action!");
    }

    // Update the visit counts
    if (visitCount_ == 0 && deltaNVisits > 0) {
        map_->numberOfVisitedEntries_++;
        // Now that we've tried it at least once, we don't have to try it again.
        map_->binSequence_.remove(binNumber_);
    }
    visitCount_ += deltaNVisits;
    cout << "VISIT COUNT UPDATED: " << visitCount_ << endl;
    map_->totalVisitCount_ += deltaNVisits;
    if (visitCount_ == 0 && deltaNVisits < 0) {
        map_->numberOfVisitedEntries_--;
        // Newly unvisited and legal => must try it.
        if (isLegal_) {
            map_->binSequence_.add(binNumber_);
        }
    }

    // Update the total Q
    totalQValue_ += deltaTotalQ;

    // Update the mean Q
    FloatType oldMeanQ = meanQValue_;
    if (visitCount_ <= 0) {
        meanQValue_ = -std::numeric_limits<FloatType>::infinity();
    } else {
        meanQValue_ = totalQValue_ / visitCount_;
    }
    return meanQValue_ != oldMeanQ;
}

void DiscretizedActionMapEntry::setLegal(bool legal) {
    if (!isLegal_) {
        if (legal) {
            // illegal => legal
            isLegal_ = true;
            if (visitCount_ == 0) {
                // Newly legal and unvisited => must try it.
                map_->binSequence_.add(binNumber_);
            }
        }
    } else {
        if (!legal) {
            // legal => illegal
            isLegal_ = false;
            map_->binSequence_.remove(binNumber_);
        }
    }
}

/* ------------------- DiscretizedActionTextSerializer ------------------- */
void DiscretizedActionTextSerializer::saveActionPool(
        ActionPool const &/*actionPool*/, std::ostream &/*os*/) {
    // Do nothing - the model can create a new one!
}
std::unique_ptr<ActionPool> DiscretizedActionTextSerializer::loadActionPool(
        std::istream &/*is*/) {
    // Use the model to create a new one.
    return getSolver()->getModel()->createActionPool(getSolver());
}

void DiscretizedActionTextSerializer::saveActionMapping(
        ActionMapping const &map, std::ostream &os) {
    DiscretizedActionMap const &discMap = static_cast<DiscretizedActionMap const &>(map);
    os << discMap.getNumberOfVisitedEntries() << " visited actions with ";
    os << discMap.getNChildren() << " children; ";
    os << discMap.getTotalVisitCount() << " visits" << std::endl;

    os << "Untried (";
    for (auto it = discMap.binSequence_.begin(); it != discMap.binSequence_.end(); it++) {
        os << *it;
        if (std::next(it) != discMap.binSequence_.end()) {
            os << ", ";
        }
    }
    os << ")" << std::endl;
    std::multimap<std::pair<FloatType, FloatType>, DiscretizedActionMapEntry const *> entriesByValue;
    long visitedCount = 0;
    for (int i = 0; i < discMap.numberOfBins_; i++) {
        DiscretizedActionMapEntry const &entry = discMap.entries_[i];
        if (entry.visitCount_ > 0) {
            visitedCount++;
        }
        // An entry is saved if it has a node, or a nonzero visit count.
        if (entry.visitCount_ > 0 || entry.childNode_ != nullptr) {
            entriesByValue.emplace(
                    std::make_pair(entry.meanQValue_, entry.binNumber_), &entry);
        }
    }
    if (visitedCount != discMap.getNumberOfVisitedEntries()) {    
        debug::show_message("ERROR: incorrect number of visited entries!");        
    }

    for (auto it = entriesByValue.rbegin(); it != entriesByValue.rend(); it++) {
        DiscretizedActionMapEntry const &entry = *it->second;
        os << "Action " << entry.getBinNumber() << " (";
        saveAction(entry.getAction().get(), os);
        os << "): " << entry.getMeanQValue() << " from ";
        os << entry.getVisitCount() << " visits; total: ";
        os << entry.getTotalQValue();
        if (!entry.isLegal()) {
            os << " ILLEGAL";
        }
        ActionNode *node = entry.getActionNode();
        if (node == nullptr) {
            os << " NO CHILD" << std::endl;
        } else {
            os << std::endl;
            save(*entry.getActionNode(), os);
        }
    }
}

std::unique_ptr<ActionMapping>
DiscretizedActionTextSerializer::loadActionMapping(BeliefNode *owner, std::istream &is) {
    std::unique_ptr<DiscretizedActionMap> discMap = std::make_unique<DiscretizedActionMap>(
            owner,
            static_cast<DiscretizedActionPool *>(getSolver()->getActionPool()),
            std::vector<long> { });
    loadActionMapping(*discMap, is);
    return std::move(discMap);
}

void DiscretizedActionTextSerializer::loadActionMapping(DiscretizedActionMap &discMap,
        std::istream &is) {

    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream sstr4(line);

    sstr4 >> discMap.numberOfVisitedEntries_ >> tmpStr >> tmpStr >> tmpStr;
    sstr4 >> discMap.nChildren_ >> tmpStr;
    sstr4 >> discMap.totalVisitCount_;

    std::getline(is, line);
    std::istringstream sstr(line);
    std::getline(sstr, tmpStr, '(');
    std::getline(sstr, tmpStr, ')');
    if (tmpStr != "") {
        std::istringstream sstr2(tmpStr);
        std::string actionString;
        while (!sstr2.eof()) {
            std::getline(sstr2, actionString, ',');
            long code;
            std::istringstream(actionString) >> code;
            discMap.binSequence_.add(code);
        }
    }

    for (long i = 0; i < discMap.numberOfVisitedEntries_ ; i++) {
        // The first line contains info from the mapping.
        std::getline(is, line);
        std::istringstream sstr2(line);
        long binNumber;
        sstr2 >> tmpStr >> binNumber;
        std::getline(sstr2, tmpStr, '(');
        std::getline(sstr2, tmpStr, ')');
        std::getline(sstr2, tmpStr, ':');
        FloatType meanQValue, totalQValue;
        long visitCount;
        std::string legalString;
        sstr2 >> meanQValue >> tmpStr;
        sstr2 >> visitCount >> tmpStr >> tmpStr;
        sstr2 >> totalQValue >> legalString;

        bool hasChild = true;
        std::string tmpStr1, tmpStr2;
        sstr2 >> tmpStr1 >> tmpStr2;
        if (tmpStr1 == "NO" && tmpStr2 == "CHILD") {
            hasChild = false;
        }

        // Create an entry to hold the action node.
        DiscretizedActionMapEntry &entry = discMap.entries_[binNumber];
        entry.binNumber_ = binNumber;
        entry.map_ = &discMap;
        entry.meanQValue_ = meanQValue;
        entry.visitCount_ = visitCount;
        entry.totalQValue_ = totalQValue;
        entry.isLegal_ = (legalString != "ILLEGAL");

        // Read in the action node itself.
        if (hasChild) {
            entry.childNode_ = std::make_unique<ActionNode>(&entry);
            ActionNode *node = entry.childNode_.get();
            load(*node, is);
        }
    }

    // Any bins we are supposed to try must be considered legal.
    for (long binNumber : discMap.binSequence_) {
        discMap.entries_[binNumber].isLegal_ = true;
    }
}

} /* namespace abt */



