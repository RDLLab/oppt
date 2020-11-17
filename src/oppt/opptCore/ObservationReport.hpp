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
#ifndef __OBSERVATION_REPORT_HPP_
#define __OBSERVATION_REPORT_HPP_
#include "typedefs.hpp"
#include "OpptUserData.hpp"

namespace oppt
{
  
/**
 * Datastructure that represents an observation request, i.e. a state an an action
 * for which an observation has to be received
 */
struct ObservationRequest {
    _NO_COPY_BUT_MOVE(ObservationRequest)
    ObservationRequest() = default;

    ~ObservationRequest() = default;
    
    /**
     * @brief The state for which the observation has to be obtained
     */
    RobotStateSharedPtr currentState = nullptr;
    
    /**
     * @brief The action for which the observation has to be obtained
     */
    const Action *action = nullptr;
    
    /**
     * @brief The error vector that is applied during the propagation
     * Usually this vector is empty, meaning that errors should be sampled
     * inside the oppt::ObservationPlugin::getObservation method
     */
    VectorFloat errorVector;
    
    bool enforceConstraints = true;
    
    bool normalizedInput = true;

    /**
     * @brief Contains any additional user data
     */
    OpptUserDataSharedPtr userData = nullptr;    
};

/**
 * Datastructure that represents an observation report. Contains an observation that was
 * received for a state and an action
 */
struct ObservationResult {
    _NO_COPY_BUT_MOVE(ObservationResult)
    ObservationResult() = default;

    ~ObservationResult() = default;
    
    /**
     * @brief The state for which the observation was obtained
     */
    RobotState const* state = nullptr;
    
    /**
     * @brief The action for which the observation was obtained
     */
    Action const* action = nullptr;
    
    /**
     * @brief The observation that was obtained
     */
    ObservationSharedPtr observation;
    
    VectorFloat errorVector;
    
    /**
     * @brief Contains any additional user data
     */
    OpptUserDataSharedPtr userData = nullptr;
};

}

#endif