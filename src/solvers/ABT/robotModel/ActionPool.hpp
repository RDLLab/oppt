#ifndef __ROBOT_ACTION_POOL_HPP__
#define __ROBOT_ACTION_POOL_HPP__
#include "solvers/ABT/solver/mappings/actions/enumerated_actions.hpp"
#include "oppt/robotHeaders/ActionSpace.hpp"
#include "RobotAction.hpp"

namespace robot
{
class RobotDiscreteActionTextSerializer;
}

namespace oppt
{

class RobotDiscretizedActionMapEntry;

class RobotDiscreteActionMapping: public abt::DiscretizedActionMap
{
public:
    friend class RobotDiscretizedActionMapEntry;
    friend class robot::RobotDiscreteActionTextSerializer;
    RobotDiscreteActionMapping(abt::BeliefNode* owner,
                               abt::DiscretizedActionPool* pool,
                               std::vector<long> binSequence);
    virtual ~RobotDiscreteActionMapping() {}

    const oppt::LinkedHashSet<long>* getBinSequence() const;

    RobotDiscretizedActionMapEntry* getEntries() const;

    long getClosestBin(abt::Action const& action) const;

    long getNumBins() const;
    
    long getNumVisitedEntries() const;

    void reduceNumVisitedEntries();

    void increaseVisitCount(long delta);

    void increaseNumVisitedEntries();

    virtual abt::ActionNode* getActionNode(abt::Action const& action) const override;

    virtual abt::ActionNode* createActionNode(abt::Action const& action) override;

    virtual void deleteChild(abt::ActionMappingEntry const* entry) override;

    virtual std::vector<abt::ActionMappingEntry const*> getChildEntries() const override;

    virtual std::vector<abt::ActionMappingEntry const*> getVisitedEntries() const override;

    virtual abt::ActionMappingEntry* getEntry(abt::Action const& action) override;

    virtual abt::ActionMappingEntry const* getEntry(abt::Action const& action) const override;

private:
    std::unique_ptr<RobotDiscretizedActionMapEntry[]> mapEntries_;

    void increaseNChildren();
    
    void addToBinSequence(long &code);

};

class RobotDiscretizedActionMapEntry: public abt::DiscretizedActionMapEntry
{
public:
    friend class RobotDiscreteActionMapping;
    friend class robot::RobotDiscreteActionTextSerializer;

    RobotDiscretizedActionMapEntry();

    virtual ~RobotDiscretizedActionMapEntry() = default;
    _NO_COPY_OR_MOVE(RobotDiscretizedActionMapEntry);

    virtual long getVisitCount() const override;

    virtual bool update(long deltaNVisits, FloatType deltaTotalQ) override;

    void reduceNumVisits();
    
private:
    void setBinNumber(long &binNumber);
    
    void setActionMapping(abt::DiscretizedActionMap *mapping);
    
    void setMeanQValue(const FloatType &meanQValue);
    
    void setVisitCount(const long &visitCount);
    
    void setTotalQValue(const FloatType &totalQValue);
    
    void setLegal(const bool &legal);
};

class RobotEnumeratedActionPool: public abt::EnumeratedActionPool
{
public:
    RobotEnumeratedActionPool(abt::Model* model,
                              const unsigned int& numStepsPerDimension,
                              const unsigned int& numActions,                              
                              std::vector<std::unique_ptr<abt::DiscretizedPoint>> allActions);
    virtual ~RobotEnumeratedActionPool() {}

    virtual std::unique_ptr<abt::ActionMapping> createActionMapping(abt::BeliefNode* node) override;

    std::vector<long> createBinSequence(abt::BeliefNode* node);

private:
    RandomGenerator* randGen_;

    unsigned int numStepsPerDimension_;

    unsigned int numActions_;

};

}

#endif
