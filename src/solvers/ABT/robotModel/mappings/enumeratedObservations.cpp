#include "enumeratedObservations.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"
#include "solvers/ABT/robotModel/belief/BeliefNode.hpp"

namespace oppt
{

/*------------------EnumeratedObservationPool------------------*/
EnumeratedABTObservationPool::EnumeratedABTObservationPool(abt::Solver* solver,
        std::vector<std::unique_ptr<abt::DiscretizedPoint>> observations):    
    solver_(solver),
    observations_(std::move(observations))
{
    
}

std::unique_ptr<abt::ObservationMapping>
EnumeratedABTObservationPool::createObservationMapping(abt::ActionNode* owner)
{    
    return std::make_unique<EnumeratedABTObservationMap>(owner, solver_, observations_);
}

/*------------------EnumeratedABTObservationMap------------------*/
EnumeratedABTObservationMap::EnumeratedABTObservationMap(abt::ActionNode* owner,
        abt::Solver* solver,
        const std::vector<std::unique_ptr<abt::DiscretizedPoint>>& allObservations):
    abt::ObservationMapping(owner),
    solver_(solver),
    allObservations_(allObservations),
    nObservations_(allObservations.size()),
    entries_(std::make_unique<EnumeratedABTObservationMapEntry[]>(nObservations_)),
    nChildren_(0),
    totalVisitCount_(0)
{    
    for (int i = 0; i < nObservations_; i++) {
        entries_[i].map_ = this;
        entries_[i].index_ = i;
        entries_[i].visitCount_ = 0;
    }
}

abt::BeliefNode* EnumeratedABTObservationMap::getBelief(
    abt::Observation const& obs) const
{    
    long code = static_cast<abt::DiscretizedPoint const&>(obs).getBinNumber();
    return entries_[code].getBeliefNode();
}

abt::BeliefNode* EnumeratedABTObservationMap::createBelief(abt::Observation const& obs)
{
    long code = static_cast<abt::DiscretizedPoint const&>(obs).getBinNumber();
    EnumeratedABTObservationMapEntry& entry = entries_[code];
    entry.childNode_ = std::make_unique<oppt::BeliefNode>(&entry, solver_);
    nChildren_++;
    return entry.getBeliefNode();
}

long EnumeratedABTObservationMap::getNChildren() const
{
    return nChildren_;
}

void EnumeratedABTObservationMap::deleteChild(abt::ObservationMappingEntry const* entry)
{
    totalVisitCount_ -= entry->getVisitCount(); // Negate the visit count.
    // Now delete the child node.    
    const_cast<EnumeratedABTObservationMapEntry&>(
        static_cast<EnumeratedABTObservationMapEntry const&>(*entry)).childNode_ = nullptr;
}

std::vector<abt::ObservationMappingEntry const*> EnumeratedABTObservationMap::getChildEntries() const
{
    std::vector<abt::ObservationMappingEntry const*> returnEntries;
    for (int i = 0; i < nObservations_; i++) {
        if (entries_[i].childNode_ != nullptr) {
            returnEntries.push_back(&entries_[i]);
        }
    }
    return returnEntries;
}
abt::ObservationMappingEntry* EnumeratedABTObservationMap::getEntry(abt::Observation const& obs)
{    
    long code = static_cast<abt::DiscretizedPoint const&>(obs).getBinNumber();
    return &entries_[code];
}
abt::ObservationMappingEntry const* EnumeratedABTObservationMap::getEntry(abt::Observation const& obs) const
{
    long code = static_cast<abt::DiscretizedPoint const&>(obs).getBinNumber();
    return &entries_[code];
}

long EnumeratedABTObservationMap::getTotalVisitCount() const
{
    return totalVisitCount_;
}

/*------------------EnumeratedABTObservationMapEntry------------------*/
abt::ObservationMapping* EnumeratedABTObservationMapEntry::getMapping() const
{
    return map_;
}

std::unique_ptr<abt::Observation> EnumeratedABTObservationMapEntry::getObservation() const
{
    //EnumeratedObservationMap const &enumMap = static_cast<EnumeratedObservationMap const &>(*map_);
    return map_->allObservations_[index_]->copy();
}
abt::BeliefNode* EnumeratedABTObservationMapEntry::getBeliefNode() const
{
    return childNode_.get();
}

long EnumeratedABTObservationMapEntry::getVisitCount() const
{
    return visitCount_;
}

void EnumeratedABTObservationMapEntry::updateVisitCount(long deltaNVisits)
{
    visitCount_ += deltaNVisits;
    map_->totalVisitCount_ += deltaNVisits;
}


}

