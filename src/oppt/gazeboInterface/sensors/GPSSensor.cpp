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
#include "GPSSensor.hpp"

namespace oppt
{
GPSSensor::GPSSensor(const gazebo::sensors::SensorPtr& sensor): GazeboSensor(sensor)
{

}

VectorFloat GPSSensor::getObservation() const
{
    VectorFloat reading(3);
    gazebo::sensors::GpsSensor* gpsSensor = static_cast<gazebo::sensors::GpsSensor*>(sensor_.get());
    reading[0] = gpsSensor->Latitude().Radian();
    reading[1] = gpsSensor->Longitude().Radian();
    reading[2] = gpsSensor->Altitude();
    return reading;
}

void GPSSensor::getObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits) const
{
    lowerLimits = VectorFloat(3);
    upperLimits = VectorFloat(3);
    lowerLimits[0] = -GZAngle::Pi.Radian();
    lowerLimits[1] = -GZAngle::Pi.Radian();
    lowerLimits[2] = -100;    
    upperLimits[0] = GZAngle::Pi.Radian();
    upperLimits[1] = GZAngle::Pi.Radian();
    upperLimits[2] = 100;
}

}
