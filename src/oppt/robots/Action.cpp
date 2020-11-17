/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
#include "oppt/robotHeaders/Action.hpp"
#include "oppt/global.hpp"

namespace oppt
{
VectorAction::VectorAction(VectorFloat& actionValues):
    Action(),
    actionVec_(actionValues),
    binNumber_(std::numeric_limits<long>::max() - 1)
{

}

VectorAction::VectorAction(const VectorFloat& actionValues):
    Action(),
    actionVec_(actionValues)
{

}

ActionUniquePtr VectorAction::copy() const
{
    ActionUniquePtr copiedAction(new VectorAction(actionVec_));
    return copiedAction;
}

void VectorAction::print(std::ostream& os) const
{
    for (size_t i = 0; i != actionVec_.size(); ++i) {
        os << actionVec_[i];
        if (i != actionVec_.size() - 1)
            os << " ";
    }    
}

void VectorAction::serialize(std::ostream& os, const std::string& prefix) const
{
    if (prefix != "") {
        os << prefix << ": ";
    }

    print(os);
    os << " END ";
}

bool VectorAction::equals(const Action& otherAction) const
{
    VectorFloat otherActionVec = static_cast<const VectorAction&>(otherAction).asVector();
    for (size_t i = 0; i < otherActionVec.size(); i++) {
        if (actionVec_[i] != otherActionVec[i]) {
            return false;
        }
    }

    return true;
}

std::size_t VectorAction::hash() const
{
    size_t hashValue = 0;
    for (auto & k : actionVec_) {
        oppt::hash_combine(hashValue, k);
    }

    return hashValue;
}

FloatType VectorAction::distanceTo(const Action& otherAction) const
{
    VectorFloat otherActionVec = static_cast<const VectorAction&>(otherAction).asVector();
    FloatType distance = 0.0;
    for (size_t i = 0; i < otherActionVec.size(); i++) {
        distance += std::pow(actionVec_[i] - otherActionVec[i], 2);
    }

    return std::sqrt(distance);
}

FloatType VectorAction::distanceTo(const oppt::ActionSharedPtr& action) const
{
    VectorFloat otherActionVec = static_cast<oppt::VectorAction*>(action.get())->asVector();
    FloatType distance = 0.0;
    for (size_t i = 0; i < otherActionVec.size(); i++) {
        distance += std::pow(actionVec_[i] - otherActionVec[i], 2);
    }

    return std::sqrt(distance);
}

VectorFloat VectorAction::asVector() const
{
    return actionVec_;
}


DiscreteVectorAction::DiscreteVectorAction(VectorFloat& actionValues):
    VectorAction(actionValues)
{

}

DiscreteVectorAction::DiscreteVectorAction(const VectorFloat& actionValues):
    VectorAction(actionValues)
{

}

long DiscreteVectorAction::getBinNumber() const
{
    return binNumber_;
}

void DiscreteVectorAction::setBinNumber(long binNumber)
{
    binNumber_ = binNumber;
}

}

