#include "RobotTransitionParameters.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace shared
{

RobotTransitionParameters::RobotTransitionParameters(oppt::PropagationResultSharedPtr& propagationResult) :
    propagationResult_(propagationResult),
    approximateDynamics_(false)
{

}

RobotTransitionParameters::RobotTransitionParameters(const oppt::PropagationResultSharedPtr& propagationResult):
    propagationResult_(propagationResult),
    approximateDynamics_(false)
{

}


oppt::PropagationResultSharedPtr RobotTransitionParameters::getPropagationResult() const
{
    return propagationResult_;
}

void RobotTransitionParameters::print(std::ostream& os) const
{
    if (propagationResult_->collisionReport && propagationResult_->collisionReport->collides) {
        os << "Collision detected: True";
    } else {
        os << "Collision detected: False";
    }
}

void RobotTransitionParameters::serialize(std::ostream& os, const std::string prefix) const
{
    if (propagationResult_->collisionReport && propagationResult_->collisionReport->collides) {
        os << "Collision detected: True\n";
    } else {
        os << "Collision detected: False\n";
    }
}

std::unique_ptr< abt::TransitionParameters > RobotTransitionParameters::copy() const
{
    return std::make_unique<RobotTransitionParameters>(propagationResult_);
}

const bool RobotTransitionParameters::approximateDynamics() const
{
    return approximateDynamics_;
}



}
