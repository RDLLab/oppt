#include "approximateObservations.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"
#include "solvers/ABT/solver/BeliefTree.hpp"
#include "solvers/ABT/solver/abstract-problem/Model.hpp"
#include "solvers/ABT/solver/abstract-problem/Observation.hpp"
#include "solvers/ABT/robotModel/belief/BeliefNode.hpp"

namespace oppt
{

ApproximateABTObservationPool::ApproximateABTObservationPool(abt::Solver* solver, FloatType maxDistance):
    abt::ApproximateObservationPool(solver, maxDistance)    
{

}

std::unique_ptr<abt::ObservationMapping> ApproximateABTObservationPool::createObservationMapping(
    abt::ActionNode* owner)
{
    return std::make_unique<ApproximateABTObservationMap>(owner, solver_, maxDistance_);
}

ApproximateABTObservationMap::ApproximateABTObservationMap(abt::ActionNode* owner, abt::Solver* solver, FloatType maxDistance):
    abt::ApproximateObservationMap(owner, solver, maxDistance)
{

}

abt::BeliefNode* ApproximateABTObservationMap::createBelief(const abt::Observation& obs)
{    
    std::unique_ptr<ApproximateABTObservationMapEntry> entry = (
                std::make_unique<ApproximateABTObservationMapEntry>());
    entry->map_ = this;
    entry->observation_ = obs.copy();
    entry->childNode_ = std::make_unique<oppt::BeliefNode>(entry.get(), solver_);
    abt::BeliefNode *node = entry->childNode_.get();
    entries_.push_back(std::move(entry));
    return node;
}

void ApproximateABTObservationMap::setTotalVisitCount(long int totalVisitCount)
{
    totalVisitCount_ = totalVisitCount;
}

ApproximateABTObservationMapEntry::~ApproximateABTObservationMapEntry() {
    
}

void ApproximateABTObservationMapEntry::setVisitCount(long int& visitCount)
{
    visitCount_ = visitCount;
}

void ApproximateABTObservationMapEntry::setMapping(ApproximateABTObservationMap *mapping)
{
    map_ = mapping;
}

void ApproximateABTObservationMapEntry::setObservation(std::unique_ptr< abt::Observation > observation)
{
    observation_ = std::move(observation);
}

void ApproximateABTObservationMapEntry::setChildNode(std::unique_ptr< abt::BeliefNode > child)
{
    childNode_ = std::move(child);
}

void ApproximateABTObservationMap::addToEntries(std::unique_ptr< abt::ApproximateObservationMapEntry > entry)
{
    entries_.push_back(std::move(entry));    
}

}