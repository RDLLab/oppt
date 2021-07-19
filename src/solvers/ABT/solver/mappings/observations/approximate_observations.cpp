/** @file approximate_observations.cpp
 *
 * Contains the implementations of the classes for approximate observation mappings.
 */
#include "approximate_observations.hpp"


#include "oppt/global.hpp"

#include <algorithm>
#include <memory>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"

#include "ObservationPool.hpp"
#include "ObservationMapping.hpp"

using std::cout;
using std::endl;

namespace abt {
/* --------------------- ApproximateObservationPool --------------------- */
ApproximateObservationPool::ApproximateObservationPool(Solver *solver, FloatType maxDistance) :
        solver_(solver),
        maxDistance_(maxDistance) {
}

std::unique_ptr<ObservationMapping> ApproximateObservationPool::createObservationMapping(
        ActionNode *owner) {
    return std::make_unique<ApproximateObservationMap>(owner, solver_, maxDistance_);
}

/* ---------------------- ApproximateObservationMap ---------------------- */
ApproximateObservationMap::ApproximateObservationMap(ActionNode *owner, Solver *solver,
        FloatType maxDistance) :
        ObservationMapping(owner),
        solver_(solver),
        maxDistance_(maxDistance),
        entries_(),
        totalVisitCount_(0) {
}

BeliefNode* ApproximateObservationMap::getBelief(Observation const &obs) const {
    ObservationMappingEntry const *entry = getEntry(obs);
    if (entry == nullptr) {
        return nullptr;
    }
    return entry->getBeliefNode();
}
BeliefNode* ApproximateObservationMap::createBelief(const Observation& obs) {
    std::unique_ptr<ApproximateObservationMapEntry> entry = (
            std::make_unique<ApproximateObservationMapEntry>());
    entry->map_ = this;
    entry->observation_ = obs.copy();
    entry->childNode_ = std::make_unique<BeliefNode>(entry.get(), solver_);
    BeliefNode *node = entry->childNode_.get();
    entries_.push_back(std::move(entry));
    return node;
}
long ApproximateObservationMap::getNChildren() const {
    return entries_.size();
}

void ApproximateObservationMap::deleteChild(ObservationMappingEntry const *entry) {
    totalVisitCount_ -= entry->getVisitCount(); // Negate the visit count
    int lastEntryNo = entries_.size() - 1;
    for (int i = 0; i < lastEntryNo; i++) {
        ApproximateObservationMapEntry *otherEntry = entries_[i].get();
        if (entry == otherEntry) {
            // If it isn't the last entry, we put the last entry in its place.
            entries_[i] = std::move(entries_[lastEntryNo]);
            break;
        }
    }
    // Remove the last entry.
    entries_.pop_back();
}

std::vector<ObservationMappingEntry const *> ApproximateObservationMap::getChildEntries() const {
    std::vector<ObservationMappingEntry const *> returnEntries;
    for (std::unique_ptr<ApproximateObservationMapEntry> const &entry : entries_) {
        returnEntries.push_back(entry.get());
    }
    return returnEntries;
}
ObservationMappingEntry *ApproximateObservationMap::getEntry(Observation const &obs) {
    ApproximateObservationMap const * constThis = const_cast<ApproximateObservationMap const *>(this);
    ObservationMappingEntry const *result = constThis->getEntry(obs);
    return const_cast<ObservationMappingEntry *>(result);
}
ObservationMappingEntry const *ApproximateObservationMap::getEntry(Observation const &obs) const {    
    FloatType shortestDistance = maxDistance_;    
    ApproximateObservationMapEntry const *bestEntry = nullptr;    
    for (std::unique_ptr<ApproximateObservationMapEntry> const &entry : entries_) {
        FloatType distance = entry->observation_->distanceTo(obs); 
        if (distance <= shortestDistance) {
            shortestDistance = distance;
            bestEntry = entry.get();	   
        }
    }

    /**if (!bestEntry) {
        cout << "=======================================" << endl;
        for (std::unique_ptr<ApproximateObservationMapEntry> const &entry : entries_) {
            cout << "--------------------------" << endl;
            cout << "entry obs: " << *(entry->observation_.get()) << endl;
            cout << "obs: " << obs << endl;
            FloatType distance = entry->observation_->distanceTo(obs); 
            cout << "dist: " << distance << endl; 
        }
    }*/

    return bestEntry;
}

long ApproximateObservationMap::getTotalVisitCount() const {
    return totalVisitCount_;
}

/* ----------------- ApproximateObservationMapEntry ----------------- */
ObservationMapping *ApproximateObservationMapEntry::getMapping() const {
    return map_;
}
std::unique_ptr<Observation> ApproximateObservationMapEntry::getObservation() const {
    return observation_->copy();
}
BeliefNode *ApproximateObservationMapEntry::getBeliefNode() const {
    return childNode_.get();
}
long ApproximateObservationMapEntry::getVisitCount() const {
    return visitCount_;
}

void ApproximateObservationMapEntry::updateVisitCount(long deltaNVisits) {
    visitCount_ += deltaNVisits;
    map_->totalVisitCount_ += deltaNVisits;
}

/* ----------------- ApproximateObservationTextSerializer ----------------- */
void ApproximateObservationTextSerializer::saveObservationPool(
        ObservationPool const &/*observationPool*/, std::ostream &/*os*/) {
    // We won't bother writing the pool to file as the model can make a new one.
}
std::unique_ptr<ObservationPool> ApproximateObservationTextSerializer::loadObservationPool(
        std::istream &/*is*/) {
    // Here we just create a new one.
    return getSolver()->getModel()->createObservationPool(getSolver());
}

void ApproximateObservationTextSerializer::saveObservationMapping(
        ObservationMapping const &map, std::ostream &os) {
    ApproximateObservationMap const &approxMap =
            (static_cast<ApproximateObservationMap const &>(map));
    os << approxMap.getNChildren() << " observation children; ";
    os << approxMap.getTotalVisitCount() << " visits {" << std::endl;
    std::vector<std::string> lines;
    for (std::unique_ptr<ApproximateObservationMapEntry> const &entry : approxMap.entries_) {
        std::ostringstream sstr;
        sstr << "\t";
        saveObservation(entry->observation_.get(), sstr);
        sstr << " -> NODE " << entry->childNode_->getId();
        sstr << "; " << entry->visitCount_ << " visits";
        sstr << std::endl;
        lines.push_back(sstr.str());
    }
    // Sort the lines so that I/O is 1:1
    std::sort(lines.begin(), lines.end());
    for (std::string line : lines) {
        os << line;
    }
    os << "}" << std::endl;
}

std::unique_ptr<ObservationMapping> ApproximateObservationTextSerializer::loadObservationMapping(
        ActionNode *owner, std::istream &is) {
    std::unique_ptr<ObservationMapping> map(
            getSolver()->getObservationPool()->createObservationMapping(owner));
    ApproximateObservationMap &approxMap =
            (static_cast<ApproximateObservationMap &>(*map));
    std::string line;
    std::getline(is, line);
    std::string tmpStr;
    std::istringstream totalsStream(line);
    long nChildren;
    totalsStream >> nChildren >> tmpStr >> tmpStr;
    totalsStream >> approxMap.totalVisitCount_;

    for (int i = 0; i < nChildren; i++) {
        std::getline(is, line);
        std::istringstream entryStream(line);
        std::unique_ptr<Observation> obs = loadObservation(entryStream);

        entryStream >> tmpStr >> tmpStr;
        std::getline(entryStream, tmpStr, ';');
        long childId;
        std::istringstream(tmpStr) >> childId;
        long visitCount;
        entryStream >> visitCount;

        // Create the entry with appropriate values.
        std::unique_ptr<ApproximateObservationMapEntry> entry = (
                std::make_unique<ApproximateObservationMapEntry>());
        entry->map_ = &approxMap;
        entry->observation_ = std::move(obs);
        entry->visitCount_ = visitCount;
        entry->childNode_ = std::make_unique<BeliefNode>(childId, entry.get(), getSolver());

        // Add the entry to the vector.
        approxMap.entries_.push_back(std::move(entry));
    }
    // Read the last line for the closing brace.
    std::getline(is, line);
    return std::move(map);
}
} /* namespace abt */

