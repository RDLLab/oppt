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
#ifndef __PROPAGATION_REPORT_HPP_
#define __PROPAGATION_REPORT_HPP_
#include "typedefs.hpp"
#include "OpptUserData.hpp"

namespace oppt
{
   
/**
 * Datastructure that represent a propagation request, i.e. the current state and
 * the action that has to be applied to the state
 */
struct PropagationRequest {
    _NO_COPY_BUT_MOVE(PropagationRequest)

    PropagationRequest() = default;

    ~PropagationRequest() = default;    
    
    /**
     * @brief The state that has to be propagated
     */
    RobotStateSharedPtr currentState = nullptr;
    
    /**
     * @brief The action used for propagating the state
     */
    const Action *action;
    
    /**
     * @brief The error vector that is applied during the propagation
     * Usually this vector is empty, meaning that errors should be sampled
     * inside the oppt::TransitionPlugin::propagateState method
     */
    VectorFloat errorVector;
    
    bool enableCollision = true;
    
    bool allowCollisions = false;
    
    /**
     * @brief Contains any additional user data
     */
    OpptUserDataSharedPtr userData = nullptr;
};

/**
 * Datastructure that represent a propagation, i.e. the previous state, the action,
 * and the next state that results from applying the action to the previous state
 */
struct PropagationResult {
    _NO_COPY_BUT_MOVE(PropagationResult)

    PropagationResult() = default;

    ~PropagationResult() = default;
    
    /** 
     * @brief The previous state 
     */
    RobotState const* previousState = nullptr;
    
    /** 
     * @brief The action that was executed 
     */
    Action const *action = nullptr;
    
    /** 
     * @brief The state that resulted from applying the action to the previous state 
     */
    RobotStateSharedPtr nextState = nullptr;
    
    /**
     * @brief The error vector that was sampled during propagation (if any)
     */
    VectorFloat errorVector;

    CollisionReportSharedPtr collisionReport = nullptr;
    
    /**
     * @brief Contains any additional user data
     */
    OpptUserDataSharedPtr userData = nullptr;
};

}

#endif
