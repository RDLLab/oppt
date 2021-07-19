#include "RobotAction.hpp"

using namespace oppt;

namespace robot
{

ContActionConstructionData::ContActionConstructionData(const FloatType* constructionDataVector, size_t dataSize):
    storage(nullptr),
    vectorData_(dataSize, 0)
{
    for (size_t i = 0; i != dataSize; i++) {
        vectorData_[i] = *(constructionDataVector + i);
    }

    storage = std::make_shared<oppt::VectorAction>(vectorData_);
}

ContActionConstructionData::ContActionConstructionData(oppt::ActionSharedPtr& action):
    storage(action),
    vectorData_(action->as<VectorAction>()->asVector())
{

}

ContActionConstructionData::ContActionConstructionData(const oppt::ActionSharedPtr& action):
    storage(action),
    vectorData_(action->as<const VectorAction>()->asVector())
{

}

const FloatType* ContActionConstructionData::data() const
{
    return vectorData_.data();    
}

size_t ContActionConstructionData::size() const
{
    return static_cast<oppt::VectorAction*>(storage.get())->asVector().size();
}

FloatType& ContActionConstructionData::operator[](size_t index)
{    
    return static_cast<oppt::VectorAction*>(storage.get())->asVector()[index];
}

const FloatType& ContActionConstructionData::operator[](size_t index) const
{    
    return static_cast<oppt::VectorAction*>(storage.get())->asVector()[index];
}

oppt::ActionSharedPtr ContActionConstructionData::getAction() const
{
    return storage;
}

size_t ContActionConstructionData::hash(const HashEqualOptions& options) const
{
    return storage->hash();
}

bool ContActionConstructionData::equal(const ContActionConstructionData& other, const HashEqualOptions& options) const
{
    oppt::ActionSharedPtr otherAction = other.getAction();
    return storage->equals(*(otherAction));
}

FloatType ContActionConstructionData::snapToTolerance(const FloatType value, const FloatType tolerance)
{
    return std::round(value / tolerance) * tolerance;
}

ContinuousRobotAction::ContinuousRobotAction(oppt::ActionSharedPtr& action):
    storage_(std::make_shared<ConstructionData>(action))
{

}

ContinuousRobotAction::ContinuousRobotAction(const oppt::ActionSharedPtr& action):
    storage_(std::make_shared<ConstructionData>(action))
{

}

ContinuousRobotAction::ContinuousRobotAction(const FloatType* constructionDataVector, size_t& dim):
    storage_(std::make_shared<ConstructionData>(constructionDataVector, dim))
{

}

bool ContinuousRobotAction::equals(const Point& otherPoint) const
{
    oppt::ActionSharedPtr otherAction = static_cast<ContinuousRobotAction const&>(otherPoint).getOpptAction();
    return storage_->getAction()->equals(*(otherAction.get()));
}

std::size_t ContinuousRobotAction::hash() const
{
    return storage_->getAction()->hash();
}

FloatType ContinuousRobotAction::distanceTo(abt::Action const& other) const
{
    oppt::ActionSharedPtr otherAction = static_cast<ContinuousRobotAction const&>(other).getOpptAction();
    return storage_->getAction()->as<VectorAction>()->distanceTo(otherAction);
}

void ContinuousRobotAction::print(std::ostream& os) const
{
    storage_->getAction()->print(os);
}

void ContinuousRobotAction::serialize(std::ostream& os, const std::string prefix) const
{
    storage_->getAction()->serialize(os, prefix);
}

std::unique_ptr<abt::Action> ContinuousRobotAction::copy() const
{
    return std::make_unique<ContinuousRobotAction>(storage_->getAction());
}

const abt::ContinuousActionConstructionDataBase& ContinuousRobotAction::getConstructionData() const
{
    return *(storage_.get());
}

const oppt::ActionSharedPtr ContinuousRobotAction::getOpptAction() const
{
    return storage_->getAction();
}

}
