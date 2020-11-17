#include "ActionPool.hpp"

namespace oppt
{
RobotDiscreteActionMapping::RobotDiscreteActionMapping(abt::BeliefNode* owner,
        abt::DiscretizedActionPool* pool,
        std::vector<long> binSequence):
    abt::DiscretizedActionMap(owner, pool, binSequence),    
    mapEntries_(std::make_unique<RobotDiscretizedActionMapEntry[]>(numberOfBins_))
{
    for (int i = 0; i < numberOfBins_; i++) {
        RobotDiscretizedActionMapEntry& entry = mapEntries_[i];
        entry.binNumber_ = i;
        entry.map_ = this;
        entry.isLegal_ = false;
    }

    // Only entries in the sequence are legal.
    for (long binNumber : binSequence_) {
        mapEntries_[binNumber].isLegal_ = true;
    }

}

RobotDiscretizedActionMapEntry* RobotDiscreteActionMapping::getEntries() const
{
    return mapEntries_.get();
}

long RobotDiscreteActionMapping::getNumBins() const
{
    return numberOfBins_;
}

const oppt::LinkedHashSet<long>* RobotDiscreteActionMapping::getBinSequence() const
{
    return &binSequence_;
}

void RobotDiscreteActionMapping::addToBinSequence(long& code)
{
    binSequence_.add(code);
}

long RobotDiscreteActionMapping::getClosestBin(abt::Action const& action) const
{
    FloatType closestDistance = 1000000000.0;
    FloatType distance = 0.0;
    long closestBin = 0;
    for (size_t i = 0; i < numberOfBins_; i++) {
        std::unique_ptr<abt::Action> entryAction = mapEntries_[i].getAction();
        distance = static_cast<robot::DiscreteRobotAction*>(entryAction.get())->distanceTo(action);
        if (distance < closestDistance) {
            closestDistance = distance;
            closestBin = i;
        }
    }

    return closestBin;
}

void RobotDiscreteActionMapping::reduceNumVisitedEntries()
{
    //if (numberOfVisitedEntries_ != 0)
    numberOfVisitedEntries_--;
}

long RobotDiscreteActionMapping::getNumVisitedEntries() const
{
    return numberOfVisitedEntries_;
}

void RobotDiscreteActionMapping::increaseVisitCount(long delta)
{
    totalVisitCount_ += delta;
}

void RobotDiscreteActionMapping::increaseNumVisitedEntries()
{
    numberOfVisitedEntries_++;
}

void RobotDiscreteActionMapping::increaseNChildren()
{
    nChildren_++;
}

abt::ActionNode* RobotDiscreteActionMapping::getActionNode(abt::Action const& action) const
{
    long code = static_cast<abt::DiscretizedPoint const&>(action).getBinNumber();
    if (code >= numberOfBins_) {
        code = getClosestBin(action);
    }

    return mapEntries_[code].getActionNode();
}

abt::ActionNode* RobotDiscreteActionMapping::createActionNode(abt::Action const& action)
{
    long code = static_cast<abt::DiscretizedPoint const&>(action).getBinNumber();
    if (code >= numberOfBins_) {
        code = getClosestBin(action);
    }

    RobotDiscretizedActionMapEntry& entry = mapEntries_[code];
    std::unique_ptr<abt::ActionNode> actionNode = std::make_unique<abt::ActionNode>(&entry);
    abt::ActionNode* node = actionNode.get();
    entry.childNode_ = std::move(actionNode);
    nChildren_++;
    return node;
}

void RobotDiscreteActionMapping::deleteChild(abt::ActionMappingEntry const* entry)
{
    RobotDiscretizedActionMapEntry& discEntry = const_cast<RobotDiscretizedActionMapEntry&>(
                static_cast<RobotDiscretizedActionMapEntry const&>(*entry));

    // Perform a negative update on the entry.
    discEntry.update(-discEntry.visitCount_, -discEntry.totalQValue_);

    // Now delete the child node.
    discEntry.childNode_ = nullptr;
}

std::vector<abt::ActionMappingEntry const*> RobotDiscreteActionMapping::getChildEntries() const
{
    std::vector<abt::ActionMappingEntry const*> returnEntries;
    for (int i = 0; i < numberOfBins_; i++) {
        RobotDiscretizedActionMapEntry const& entry = mapEntries_[i];
        if (entry.childNode_ != nullptr) {
            returnEntries.push_back(&entry);
        }
    }
    return returnEntries;
}

std::vector<abt::ActionMappingEntry const*> RobotDiscreteActionMapping::getVisitedEntries() const
{
    std::vector<abt::ActionMappingEntry const*> returnEntries;
    for (int i = 0; i < numberOfBins_; i++) {
        RobotDiscretizedActionMapEntry const& entry = mapEntries_[i];
        if (entry.visitCount_ > 0) {
            if (!entry.isLegal_) {
                debug::show_message("WARNING: Illegal entry with nonzero visit count!");
            }
            returnEntries.push_back(&entry);
        }
    }
    return returnEntries;
}

abt::ActionMappingEntry* RobotDiscreteActionMapping::getEntry(abt::Action const& action)
{
    long code = static_cast<abt::DiscretizedPoint const&>(action).getBinNumber();
    return &mapEntries_[code];
}

abt::ActionMappingEntry const* RobotDiscreteActionMapping::getEntry(abt::Action const& action) const
{
    long code = static_cast<abt::DiscretizedPoint const&>(action).getBinNumber();

    return &mapEntries_[code];
}

RobotDiscretizedActionMapEntry::RobotDiscretizedActionMapEntry():
    abt::DiscretizedActionMapEntry()
{

}

long RobotDiscretizedActionMapEntry::getVisitCount() const
{
    return visitCount_;
}

void RobotDiscretizedActionMapEntry::reduceNumVisits()
{
    //if (visitCount_ != 0)
    visitCount_--;
}

void RobotDiscretizedActionMapEntry::setBinNumber(long& binNumber)
{
    binNumber_ = binNumber;
}

bool RobotDiscretizedActionMapEntry::update(long deltaNVisits, FloatType deltaTotalQ)
{
    auto actionMapping = static_cast<RobotDiscreteActionMapping*>(getMapping());
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
        actionMapping->increaseNumVisitedEntries();
        // Now that we've tried it at least once, we don't have to try it again.
        actionMapping->binSequence_.remove(binNumber_);
        //getMapping()->binSequence_.remove(binNumber_);
    }
    visitCount_ += deltaNVisits;

    actionMapping->increaseVisitCount(deltaNVisits);
    if (visitCount_ == 0 && deltaNVisits < 0) {
        actionMapping->reduceNumVisitedEntries();
        //getMapping()->numberOfVisitedEntries_--;
        // Newly unvisited and legal => must try it.
        if (isLegal_) {
            actionMapping->binSequence_.add(binNumber_);
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

void RobotDiscretizedActionMapEntry::setActionMapping(abt::DiscretizedActionMap* mapping)
{
    map_ = mapping;
}

void RobotDiscretizedActionMapEntry::setMeanQValue(const FloatType& meanQValue)
{
    meanQValue_ = meanQValue;
}

void RobotDiscretizedActionMapEntry::setVisitCount(const long& visitCount)
{
    visitCount_ = visitCount;
}

void RobotDiscretizedActionMapEntry::setTotalQValue(const FloatType& totalQValue)
{
    totalQValue_ = totalQValue;
}

void RobotDiscretizedActionMapEntry::setLegal(const bool& legal)
{
    isLegal_ = legal;
}

RobotEnumeratedActionPool::RobotEnumeratedActionPool(abt::Model* model,
        const unsigned int& numStepsPerDimension,
        const unsigned int& numActions,        
        std::vector<std::unique_ptr<abt::DiscretizedPoint>> allActions):
    abt::EnumeratedActionPool(model, std::move(allActions)),
    randGen_(model->getRandomGenerator()),    
    numStepsPerDimension_(numStepsPerDimension),
    numActions_(numActions)
{

}

std::unique_ptr<abt::ActionMapping> RobotEnumeratedActionPool::createActionMapping(abt::BeliefNode* node)
{
    std::unique_ptr<abt::ActionMapping> actionMapping(new RobotDiscreteActionMapping(node, 
                                                                                        this, 
                                                                                        createBinSequence(node)));
    return std::move(actionMapping);
}

std::vector<long> RobotEnumeratedActionPool::createBinSequence(abt::BeliefNode* node)
{
    std::vector<long> bins(numActions_);
    for (size_t i = 0; i < numActions_; i++) {
        bins[i] = i;
    }

    std::shuffle(bins.begin(), bins.end(), *randGen_);
    return std::move(bins);
}

}

