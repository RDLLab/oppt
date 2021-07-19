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
#ifndef __GAZEBO_SENSOR_HPP__
#define __GAZEBO_SENSOR_HPP__
#include "oppt/opptCore/core.hpp"
#include <gazebo/sensors/sensors.hh>

namespace oppt
{
class GazeboSensor
{
public:
    GazeboSensor(const gazebo::sensors::SensorPtr &sensor);
    
    virtual ~GazeboSensor() {}
    
    std::string getName() const;
    
    std::string getScopedName() const;
    
    virtual VectorFloat getObservation() const = 0;
    
    virtual void getObservationLimits(VectorFloat &lowerLimits, VectorFloat &upperLimits) const = 0;
    
    gazebo::sensors::SensorPtr getRawSensor() const;
    
protected:
    gazebo::sensors::SensorPtr sensor_;

};

typedef std::shared_ptr<GazeboSensor> GazeboSensorPtr;

}

#endif
