#include "BeliefNode.hpp"
#include "solvers/ABT/robotModel/RobotAction.hpp"
#include "solvers/ABT/robotModel/RobotState.hpp"
#include "solvers/ABT/robotModel/RobotModel.hpp"
#include "solvers/ABT/solver/StatePool.hpp"
#include "solvers/ABT/solver/HistoryEntry.hpp"
#include "solvers/ABT/robotModel/solver/StateInfo.hpp"

namespace oppt
{

EvaluatedActionsMapEntry::EvaluatedActionsMapEntry():
    transitionParameters_(nullptr),
    state_(nullptr),
    propagatedState_(nullptr),
    observation_(nullptr),
    heuristic_(0),
    reward_(0),
    terminal_(false)
{
}

void EvaluatedActionsMapEntry::setTransitionParameters(const abt::TransitionParameters* transitionParameters)
{
    transitionParameters_ = static_cast<const shared::RobotTransitionParameters*>(transitionParameters)->copy();
}

void EvaluatedActionsMapEntry::setState(const abt::State* state)
{
    state_ = state->copy();
}

void EvaluatedActionsMapEntry::setPropagatedState(const abt::State* state)
{
    propagatedState_ = state->copy();
}

void EvaluatedActionsMapEntry::setObservation(const abt::Observation* observation)
{
    observation_ = observation->copy();
}

void EvaluatedActionsMapEntry::setHeuristic(const FloatType& heuristic)
{
    heuristic_ = heuristic;
}

void EvaluatedActionsMapEntry::setReward(const FloatType& reward)
{
    reward_ = reward;
}

void EvaluatedActionsMapEntry::setTerminal(const bool& terminal)
{
    terminal_ = terminal;
}

const abt::State* EvaluatedActionsMapEntry::getState() const
{
    return state_.get();
}

const abt::State* EvaluatedActionsMapEntry::getPropagatedState() const
{
    return propagatedState_.get();
}

const abt::Observation* EvaluatedActionsMapEntry::getObservation() const
{
    return observation_.get();
}

const abt::TransitionParameters* EvaluatedActionsMapEntry::getTransitionParameters() const
{
    return transitionParameters_.get();
}

const FloatType EvaluatedActionsMapEntry::getHeuristic() const
{
    return heuristic_;
}

const FloatType EvaluatedActionsMapEntry::getReward() const
{
    return reward_;
}

const bool EvaluatedActionsMapEntry::getTerminal() const
{
    return terminal_;
}

EvaluatedActionsMap::EvaluatedActionsMap():
    evaluatedActionsMap_()
{

}

bool EvaluatedActionsMap::contains(long int& key) const
{
    if (evaluatedActionsMap_.find(key) == evaluatedActionsMap_.end())
        return false;
    return true;
}

size_t EvaluatedActionsMap::size() const
{
    return evaluatedActionsMap_.size();
}

void EvaluatedActionsMap::insert(long int& key, const abt::State* state, const abt::Model::StepResult& stepResult)
{
    if (evaluatedActionsMap_.find(key) == evaluatedActionsMap_.end())
        evaluatedActionsMap_[key] = EvaluatedActionStateVector();
    std::unique_ptr<EvaluatedActionsMapEntry> entry = std::make_unique<EvaluatedActionsMapEntry>();
    entry->setState(state);
    entry->setPropagatedState(stepResult.nextState.get());
    entry->setObservation(stepResult.observation.get());
    entry->setTransitionParameters(stepResult.transitionParameters.get());
    entry->setReward(stepResult.reward);
    entry->setTerminal(stepResult.isTerminal);
    evaluatedActionsMap_.at(key).push_back(std::move(entry));
}


const EvaluatedActionStateVector* EvaluatedActionsMap::get(long int& key) const
{
    if (evaluatedActionsMap_.find(key) != evaluatedActionsMap_.end()) {
        return &(evaluatedActionsMap_.at(key));
    }
    return nullptr;
}



BeliefNode::BeliefNode(long int id, abt::ObservationMappingEntry* parentEntry, abt::Solver* solver):
    abt::BeliefNode(id, parentEntry, solver),
    totalNumDynamicsUsed_(0)
{
    
}

BeliefNode::BeliefNode(abt::Solver* solver):
    abt::BeliefNode(solver),
    totalNumDynamicsUsed_(0)
{
    
}

BeliefNode::BeliefNode(abt::ObservationMappingEntry* parentEntry, abt::Solver* solver):
    abt::BeliefNode(parentEntry, solver),
    totalNumDynamicsUsed_(0)
{
    
}

void BeliefNode::clear()
{
    for (auto &particle: particles_)
        static_cast<oppt::StateInfo *>(static_cast<abt::HistoryEntry*>(particle)->stateInfo_)->containedInBeliefs_--;  
    particles_ = oppt::RandomAccessSet<abt::HistoryEntry *>();
}

void BeliefNode::addParticle(abt::HistoryEntry* newHistEntry)
{   
    bool added = particles_.add(newHistEntry);    
    /**if (added)  {	
	static_cast<oppt::StateInfo*>(newHistEntry->stateInfo_)->containedInBeliefs_++;
    }*/
    if (newHistEntry->getId() == 0) {
        nStartingSequences_++;
    }
}

void BeliefNode::removeParticle(abt::HistoryEntry* histEntry)
{   
    bool removed = particles_.remove(histEntry);
    if (histEntry->getId() == 0) {
        nStartingSequences_--;
    } 
}

void BeliefNode::registerParticle(abt::HistoryEntry* newHistEntry)
{
    static_cast<oppt::StateInfo*>(newHistEntry->stateInfo_)->containedInBeliefs_++;
}

void BeliefNode::deregisterParticle(abt::HistoryEntry* histEntry)
{
    static_cast<oppt::StateInfo*>(histEntry->stateInfo_)->containedInBeliefs_--;
}

BeliefNode::~BeliefNode()
{   
    /**for (auto it = particles_.begin(); it != particles_.end(); ++it) {
	if (static_cast<oppt::StateInfo *>(static_cast<abt::HistoryEntry*>(*it)->stateInfo_)->containedInBeliefs_ > 0)
	    static_cast<oppt::StateInfo *>(static_cast<abt::HistoryEntry*>(*it)->stateInfo_)->containedInBeliefs_--;	
    }*/
}

unsigned int BeliefNode::getTotalNumDynamicsUsed() const
{
    return totalNumDynamicsUsed_;
}

}


