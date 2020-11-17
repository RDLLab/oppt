#include "RobotAction.hpp"

#include <cstddef>                      // for size_t
#include <cstdint>
#include <math.h>

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <iostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator
#include <boost/concept_check.hpp>

using std::cout;
using std::endl;

namespace robot
{

DiscreteRobotAction::DiscreteRobotAction(oppt::ActionSharedPtr& action) :
    action_(action)
{
    if (!action_) {
	getchar();
	ERROR("Action is nullptr1");
    }
}

DiscreteRobotAction::DiscreteRobotAction(const oppt::ActionSharedPtr& action):
    action_(action)
{
    if (!action_)
	ERROR("Action is nullptr3");
}

std::unique_ptr<abt::Action> DiscreteRobotAction::copy() const
{    
    return std::make_unique<DiscreteRobotAction>(action_);
}

FloatType DiscreteRobotAction::distanceTo(abt::Action const& other) const
{    
    oppt::ActionSharedPtr otherAction = static_cast<DiscreteRobotAction const &>(other).getOpptAction();    
    return action_->distanceTo(otherAction);    
}

void DiscreteRobotAction::print(std::ostream& os) const
{
    action_->print(os);    
}

void DiscreteRobotAction::serialize(std::ostream &os, const std::string prefix) const {
    if (!action_)
	ERROR("Action is nullptr");
    action_->serialize(os, prefix);
}

const oppt::ActionSharedPtr DiscreteRobotAction::getOpptAction() const
{
    return action_;
}

long DiscreteRobotAction::getBinNumber() const
{    
    return static_cast<oppt::DiscreteVectorAction *>(action_.get())->getBinNumber();    
}

}
