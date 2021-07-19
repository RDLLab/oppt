/** @file ManipulatorObservation.cpp
 *
 * Contains the implementations for the methods of RockSampleObservation.
 */
#include "RobotObservation.hpp"

#include <cstddef>                      // for size_t
#include <cmath>
#include <random>

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "oppt/global.hpp"

using std::cout;
using std::endl;

namespace shared
{

RobotObservation::RobotObservation(const oppt::ObservationSharedPtr& observation) :
    observation_(observation)
{

}

std::unique_ptr<abt::Observation> RobotObservation::copy() const
{
    return std::make_unique<RobotObservation>(observation_);
}

FloatType RobotObservation::distanceTo(abt::Observation const& otherObs) const
{    
    return observation_->distanceTo(*(static_cast<RobotObservation const&>(otherObs).getOpptObservation().get()));
}

bool RobotObservation::equals(abt::Observation const& otherObs) const
{    
    return observation_->equals(*(static_cast<const RobotObservation&>(otherObs).getOpptObservation().get()));
}

std::size_t RobotObservation::hash() const
{
    return observation_->hash();
}

void RobotObservation::print(std::ostream& os) const
{
    observation_->print(os);
}

void RobotObservation::serialize(std::ostream& os, const std::string prefix) const
{
    observation_->serialize(os, prefix);
}

long RobotObservation::getBinNumber() const
{    
    //long code = static_cast<oppt::DiscreteVectorObservation*>(observation_.get())->getBinNumber();    
    return static_cast<oppt::DiscreteVectorObservation*>(observation_.get())->getBinNumber();
}

oppt::ObservationSharedPtr RobotObservation::getOpptObservation() const
{
    return observation_;
}

}
/* namespace manipulator */

