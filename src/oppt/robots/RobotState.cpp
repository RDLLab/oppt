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
#include "oppt/robotHeaders/RobotState.hpp"
#include <cmath>
#include <sdf/parser.hh>
#include <math.h>
#include "oppt/global.hpp"

namespace oppt
{

bool logGazeboState = false;

void RobotState::setSubStates(const std::vector<RobotStateSharedPtr>& subStates)
{
    subStates_ = subStates;
}

const std::vector<RobotStateSharedPtr> RobotState::getSubStates() const
{
    return subStates_;
}

WeightedRobotState::WeightedRobotState():
    RobotState(),
    weight_(1.0)
{

}

FloatType WeightedRobotState::getWeight() const
{
    return weight_;
}

void WeightedRobotState::setWeight(const FloatType& weight)
{
    weight_ = weight;
}

/**
 * VectorState
 */
VectorState::VectorState(VectorFloat& stateVector):
    WeightedRobotState(),
    state_(stateVector)
{

}

VectorState::VectorState(const VectorFloat& stateVector):
    WeightedRobotState(),
    state_(stateVector)
{

}

void VectorState::serialize(std::ostream& os, const std::string prefix) const
{
    if (prefix != "") {
        os << prefix << ": ";
    }

    VectorFloat state = state_;
    for (size_t i = 0 ; i < state.size(); i++) {
        if (i < state.size() - 1) {
            os << state[i] << " ";
        } else {
            os << state[i];
        }
    }

    os << " w: " << weight_ << " END ";
    if (subStates_.size() > 0) {
        os << "SUBSTATES_BEGIN ";
        for (auto& subState : subStates_) {
            auto vec = subState->as<VectorState>()->asVector();
            os << "SS_BEGIN ";
            for (size_t i = 0; i != vec.size(); ++i) {
                if (i < state_.size() - 1) {
                    os << vec[i] << " ";
                }
            }
            os << "SS_END ";
        }
        os << "SUBSTATES_END ";
    }

    if (logGazeboState && gazeboWorldState_) {
        os << "WORLDSTATE_BEGIN ";
        /**const sdf::ElementPtr worldSDF =
            gazeboWorldState_->getWorld()->SDF();
        std::string sdfString = worldSDF->ToString("");
        sdf::ElementPtr stateElem = worldSDF->GetElement("state");
        if (!stateElem)
            ERROR("No state element");
        gazeboWorldState_->getWorldState()->FillSDF(stateElem);        
        cout << "sdfString1: " << endl;
        cout << sdfString << endl;
        cout << "===============" << endl;
        cout << "sdfString2: " << endl;
        cout << stateElem->ToString("") << endl;
        getchar();*/
        os << std::setprecision(20);
        std::string stateString = gazeboWorldState_->toString();
        stateString.erase(std::remove(stateString.begin(), stateString.end(), '\n'), stateString.end());
        os << stateString << " ";
        os << "WORLDSTATE_END ";
        os << std::setprecision(7);

    }

    os << "USER_DATA_BEGIN ";
    if (userData_) {
        os << std::setprecision(20);
        userData_->serialize(os);
        os << std::setprecision(7);
        //if (static_cast<RobotStateUserData *>(userData_.get())->previousState)
        //    os << *(static_cast<RobotStateUserData*>(userData_.get())->previousState.get()) << " ";

    }
    os << " USER_DATA_END ";
    //os << "\n";
}

void VectorState::print(std::ostream& os) const
{
    for (size_t i = 0; i < state_.size(); i++) {
        if (i < state_.size() - 1) {
            os << state_[i] << " ";
        } else {
            os << state_[i];
        }
    }
    os << " w: " << weight_;
}

FloatType VectorState::distanceTo(const RobotState& otherState) const
{
    VectorFloat otherStateVec = static_cast<const VectorState*>(&otherState)->asVector();
    FloatType sum = 0.0;
    for (size_t i = 0; i < state_.size(); i++) {
        sum += std::pow(state_[i] - otherStateVec[i], 2);
    }
    return std::sqrt(sum);
}

bool VectorState::equals(const RobotState& otherState) const
{
    if (std::fabs(static_cast<const VectorState*>(&otherState)->getWeight() - weight_) > 1e-10)
        return false;
    VectorFloat otherStateVec = static_cast<const VectorState*>(&otherState)->asVector();
    for (size_t i = 0; i < state_.size(); i++) {
        if (std::fabs(state_[i] - otherStateVec[i]) > 1e-10) {
            return false;
        }
    }

    return true;
}

std::size_t VectorState::hash() const
{
    std::size_t hashValue = 0;
    for (size_t i = 0; i < state_.size(); i++) {
        oppt::hash_combine(hashValue, state_[i]);
    }
    //oppt::hash_combine(hashValue, weight_);
    return hashValue;
}

VectorFloat VectorState::asVector() const
{
    return state_;
}

FloatType VectorState::round_(const FloatType& value) const
{

    if (value == 0.) {
        return value;
    } else if (value > 1.) {
        int ex = floor(log10(abs(value))) - roundingPrecision_ + 1;
        FloatType div = pow(10, ex);
        return floor(value / div + 0.5) * div;
    }

    FloatType po = pow(10, roundingPrecision_);
    FloatType ceil_value = ceil(value * po) / po;
    FloatType floor_value = floor(value * po) / po;
    if (fabs(value - floor_value) < fabs(ceil_value - value)) {
        return floor_value;
    }
    return ceil_value;
}

}
