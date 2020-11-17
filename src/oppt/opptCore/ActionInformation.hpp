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
#ifndef __OPPT_ACTION_SPACE_INFORMATION__
#define __OPPT_ACTION_SPACE_INFORMATION__
#include "oppt/opptCore/typedefs.hpp"
#include "SpaceInformation.hpp"

namespace oppt
{

/**
 * Datastructure that constains information about the POMDP action space. 
 * This is used to setup the oppt::GazeboInterface
 */
struct ActionSpaceInformation {
    /** @brief Contains the names of torque controlled joints */
    VectorString torqueControlledJoints;
    
    /** @brief Contains the names of velocity controlled joints */
    VectorString velocityControlledJoints;
    
    /** @brief Contains the names of position controlled joints */
    VectorString positionControlledJoints;

    /** @brief Contains the names of position increment controlled joints */
    VectorString positionIncrementControlledJoints;
    
    /** @brief Contains the number of additional action space dimensions */
    unsigned int additionalDimensions = 0;
    
    /** @brief Contains the lower and upper bounds of the additional action space dimensions*/
    std::vector<std::pair<FloatType, FloatType>> lowerUpperLimitsAdditional;

    /** @brief Contains the lower and upper bounds of the joint position increments*/
    std::vector<std::pair<FloatType, FloatType>> lowerUpperLimitsJointPositionIncrements;
    
    /** @brief Contains the redundant joints */
    std::vector<VectorString> redundantJoints;

    SpaceVariablesPtr orderedVariables;

    /** @brief Determines if the physics engine has to be used to simulation the transition dynamics */
    bool requiresPhysics = false;
};
}

#endif
