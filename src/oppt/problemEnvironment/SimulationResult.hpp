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
#ifndef _SIMULATION_RESULT_HPP_
#define _SIMULATION_RESULT_HPP_
#include "oppt/opptCore/core.hpp"

namespace oppt
{
struct SimulationResult {
    SimulationResult() = default;
    
    unsigned int stepsTaken = 0;
    
    FloatType totalPlanningTime = 0;
    
    bool successfulRun = false;
    
    FloatType discountedReward = std::numeric_limits<FloatType>::quiet_NaN();
};
}

#endif
