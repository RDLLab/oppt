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
#ifndef __DEPTH_SENSOR_HPP__
#define __DEPTH_SENSOR_HPP__
#include "GazeboSensor.hpp"

namespace oppt
{
class DepthSensor: public GazeboSensor
{
public:
    DepthSensor(const gazebo::sensors::SensorPtr& sensor);
    
    virtual ~DepthSensor() {}
    
    virtual VectorFloat getObservation() const override;
    
    virtual void getObservationLimits(VectorFloat &lowerLimits, VectorFloat &upperLimits) const override;

};

}

#endif

