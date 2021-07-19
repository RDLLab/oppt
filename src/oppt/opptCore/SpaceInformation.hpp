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
#ifndef _OPPT_SPACE_INFORMATION_HPP_
#define _OPPT_SPACE_INFORMATION_HPP_
#include "typedefs.hpp"
#include "SpaceComponents.hpp"

namespace oppt
{

struct SpaceInformation {
    _NO_COPY_BUT_MOVE(SpaceInformation)

    SpaceInformation() = default;

    ~SpaceInformation() = default;
    
    VectorString jointPositions;

    VectorString jointPositionsIncrement;
    
    VectorString jointVelocities;
    
    std::vector<VectorString> redundantJoints;
    
    VectorString containedLinkPoses;
    
    std::vector<VectorFloat> containedLinkPosesLowerLimits;
    
    std::vector<VectorFloat> containedLinkPosesUpperLimits;
    
    VectorString containedLinkPositionsX;
    
    std::vector<VectorFloat> containedLinkPositionsXLimits;
    
    VectorString containedLinkPositionsY;
    
    std::vector<VectorFloat> containedLinkPositionsYLimits;
    
    VectorString containedLinkPositionsZ;
    
    std::vector<VectorFloat> containedLinkPositionsZLimits;
    
    VectorString containedLinkOrientationsX;
    
    std::vector<VectorFloat> containedLinkOrientationsXLimits;
    
    VectorString containedLinkOrientationsY;
    
    std::vector<VectorFloat> containedLinkOrientationsYLimits;
    
    VectorString containedLinkOrientationsZ;
    
    std::vector<VectorFloat> containedLinkOrientationsZLimits;
    
    VectorString containedLinkLinearVelocitiesX;
    
    std::vector<VectorFloat> containedLinkLinearVelocitiesXLimits;
    
    VectorString containedLinkLinearVelocitiesY;
    
    std::vector<VectorFloat> containedLinkLinearVelocitiesYLimits;
    
    VectorString containedLinkLinearVelocitiesZ;
    
    std::vector<VectorFloat> containedLinkLinearVelocitiesZLimits;
    
    VectorString containedLinkVelocitiesLinear;
    
    std::vector<VectorFloat> containedLinkVelocitiesLinearLimits;
    
    VectorString containedLinkAngularVelocitiesX;
    
    std::vector<VectorFloat> containedLinkAngularVelocitiesXLimits;
    
    VectorString containedLinkAngularVelocitiesY;
    
    std::vector<VectorFloat> containedLinkAngularVelocitiesYLimits;
    
    VectorString containedLinkAngularVelocitiesZ;
    
    std::vector<VectorFloat> containedLinkAngularVelocitiesZLimits;
    
    VectorString containedLinkVelocitiesAngular;
    
    std::vector<VectorFloat> containedLinkVelocitiesAngularLimits;
    
    unsigned int additionalDimensions = 0;
    
    std::vector<VectorFloat> lowerUpperLimitsAdditional;

    SpaceVariablesPtr orderedVariables;

    //VectorString orderedVariables;
};
}

#endif
