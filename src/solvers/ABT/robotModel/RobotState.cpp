/** @file RockSampleState.cpp
 *
 * Contains the implementation for the methods of RockSampleState.
 */
#include "RobotState.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <iostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "oppt/global.hpp"

using std::cout;
using std::endl;

namespace shared
{

RobotState::RobotState(const oppt::RobotStateSharedPtr& robotState):
    abt::Vector(),
    robotState_(robotState),
    propagationResult_(nullptr)
{
}

std::unique_ptr<abt::Point> RobotState::copy() const
{
    std::unique_ptr<abt::Point> copiedState = std::make_unique<RobotState>(robotState_);
    static_cast<RobotState*>(copiedState.get())->setWeight(getWeight());
    if (propagationResult_)
        static_cast<RobotState*>(copiedState.get())->setPropagationResult(propagationResult_);
    return std::move(copiedState);
}

FloatType RobotState::distanceTo(abt::State const& otherState) const
{
    RobotState const& otherRobotState =
        static_cast<RobotState const&>(otherState);
    oppt::RobotStateSharedPtr otherOpptSate = otherRobotState.getOpptState();
    return robotState_->distanceTo(*(otherOpptSate.get()));
}

bool RobotState::equals(abt::State const& otherState) const
{
    //return this == &otherState;
    const RobotState otherRobotState = static_cast<RobotState const&>(otherState);
    oppt::RobotStateSharedPtr opptState = otherRobotState.getOpptState();
    /**if (robotState_ == otherRobotState.getOpptState().get()) {
        cout << "states are equal: " << endl;
        cout << *(robotState_.get()) << endl;
        cout << *(otherRobotState.getOpptState().get()) << endl;
        return true;
    }
    return false;*/
    /**if (robotState_->equals(opptState) && otherRobotState.getWeight() == getWeight()) {
    return true;
    }*/
    if (robotState_->equals(*(opptState.get()))) {
        //cout << "states equal:" << endl;
        //cout << *(opptState.get()) << endl;
        //cout << *(robotState_.get()) << endl;
        return true;
    }
    return false;
}

FloatType RobotState::round_(const FloatType& value, const int& precision) const
{
    FloatType ceil_value = ceil(value * pow(10, precision)) / pow(10, precision);
    FloatType floor_value = floor(value * pow(10, precision)) / pow(10, precision);
    if (fabs(value - floor_value) < fabs(ceil_value - value)) {
        return floor_value;
    }
    return ceil_value;


}

VectorFloat RobotState::asVector() const
{
    return static_cast<oppt::VectorState*>(robotState_.get())->asVector();
}

const oppt::RobotStateSharedPtr RobotState::getOpptState() const
{
    return robotState_;
}

std::size_t RobotState::hash() const
{
    return robotState_->hash();
}

void RobotState::serialize(std::ostream& os, const std::string prefix) const
{
    robotState_->serialize(os, prefix);
    /**oppt::WorldStatePtr worldState = robotState_->getWorldState();
    if (worldState) {
        worldStateSerializer_->serializeWorldState(worldState, os);
    }*/
}

void RobotState::print(std::ostream& os) const
{
    robotState_->print(os);
}

void RobotState::setWeight(const FloatType weight) const
{
    static_cast<oppt::WeightedRobotState*>(robotState_.get())->setWeight(weight);
    //weight_ = weight;
}

FloatType RobotState::getWeight() const
{
    return static_cast<oppt::WeightedRobotState*>(robotState_.get())->getWeight();
    //return weight_;
}

void RobotState::setPropagationResult(const oppt::PropagationResultSharedPtr& propagationResult)
{
    propagationResult_ = propagationResult;
}

const oppt::PropagationResultSharedPtr RobotState::getPropagationResult() const
{
    return propagationResult_;
}

} /* namespace rocksample */

