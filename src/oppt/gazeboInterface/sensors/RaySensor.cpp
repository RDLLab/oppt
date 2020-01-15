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
#include "RaySensor.hpp"

namespace oppt
{
RaySensor::RaySensor(const gazebo::sensors::SensorPtr& sensor): GazeboSensor(sensor)
{

}

VectorFloat RaySensor::getObservation() const
{
    gazebo::sensors::RaySensor* raySensor = static_cast<gazebo::sensors::RaySensor*>(sensor_.get());
    VectorFloat observation(raySensor->RayCount() * raySensor->VerticalRayCount(), 0);
    return observation;
}

void RaySensor::getObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits) const
{
    gazebo::sensors::RaySensor* raySensor = static_cast<gazebo::sensors::RaySensor*>(sensor_.get());
    lowerLimits = VectorFloat(raySensor->RayCount() * raySensor->VerticalRayCount(), raySensor->RangeMin());
    upperLimits = VectorFloat(raySensor->RayCount() * raySensor->VerticalRayCount(), raySensor->RangeMax());
}

}
