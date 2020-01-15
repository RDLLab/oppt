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
#ifndef _ROBOT_IMPL_SPACE_INFORMATION_FACTORY_
#define _ROBOT_IMPL_SPACE_INFORMATION_FACTORY_
#include "oppt/opptCore/core.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotConfigOptions.hpp"
#include "oppt/opptCore/SpaceInformation.hpp"

using namespace oppt;

namespace oppt
{
namespace SpaceInformationFactory
{

const StateSpaceInformationPtr makeStateSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface);

const ActionSpaceInformationPtr makeActionSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface);

const ObservationSpaceInformationPtr makeObservationSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface);

void checkSpaceLimits(SpaceInformation *spaceInformation, 
		      const RobotConfigOptions* robotConfigOptions, 
		      const std::string spaceType);

}
}

#endif
