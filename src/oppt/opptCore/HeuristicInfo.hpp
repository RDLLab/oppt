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
#ifndef _HEURISTIC_INFO_HPP_
#define _HEURISTIC_INFO_HPP_
#include "typedefs.hpp"
#include "logging.hpp"

namespace oppt
{
class RewardPlugin;
class HeuristicInfo
{
public:
    _NO_COPY_BUT_MOVE(HeuristicInfo)
    HeuristicInfo():
        currentBelief(nullptr),
        currentState(nullptr),        
        discountFactor(0),
        currentStep(1) {

    }

    oppt::BeliefSharedPtr currentBelief;

    oppt::RobotStateSharedPtr currentState;
    
    const Action *action = nullptr;
    
    FloatType discountFactor;

    FloatType timeout;

    unsigned int currentStep;

};

}

#endif
