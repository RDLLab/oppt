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
#ifndef _OPPT_STEP_RESULT_HPP_
#define _OPPT_STEP_RESULT_HPP_
#include "oppt/opptCore/core.hpp"

namespace oppt
{

struct StepResult {
    StepResult(const unsigned int& step, RobotEnvironment* robotEnvironment):
        step(step),
        robotEnvironment_(robotEnvironment) {

    }

    void serialize(std::ofstream& os) {
        auto robot = robotEnvironment_->getRobot();
        os << "t = " << step << endl;
        if (currentState)
            currentState->serialize(os, "S");
        //robot->getStateSpace()->denormalizeState(currentState)->serialize(os, "S");
        os << "\n";

        // Serialize the action
        if (action) {
            robot->getActionSpace()->denormalizeAction(action)->serialize(os, "A");
            os << "\n";
        }

        // Serialize the observation
        if (observation) {
            robot->getObservationSpace()->denormalizeObservation(observation)->serialize(os, "O");
            os << "\n";
        }
        os << "IMMEDIATE_REWARD: " << reward << endl;
        os << "DISCOUNT_FACTOR: " << discountFactor << endl;
        os << "DISCOUNTED_REWARD: " << discountFactor* reward << endl;
    }

    unsigned int step;

    RobotStateSharedPtr currentState = nullptr;

    ActionSharedPtr action = nullptr;

    ObservationSharedPtr observation = nullptr;

    FloatType reward = std::numeric_limits<FloatType>::quiet_NaN();

    FloatType discountFactor = 1.0;

    RobotEnvironment* robotEnvironment_ = nullptr;
};

}

#endif
