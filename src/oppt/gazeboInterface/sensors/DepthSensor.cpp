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
#include "DepthSensor.hpp"
#include <gazebo/rendering/DepthCamera.hh>

namespace oppt
{
DepthSensor::DepthSensor(const gazebo::sensors::SensorPtr& sensor): GazeboSensor(sensor)
{

}

VectorFloat DepthSensor::getObservation() const
{
    gazebo::sensors::DepthCameraSensor* depthCameraSensor = static_cast<gazebo::sensors::DepthCameraSensor*>(sensor_.get());
    unsigned int imageHeight = depthCameraSensor->DepthCamera()->ImageHeight();
    unsigned int imageWidth = depthCameraSensor->DepthCamera()->ImageWidth();
    VectorFloat observation(imageHeight * imageWidth, 0.0);
    auto dataPtr = depthCameraSensor->DepthCamera()->DepthData();
    for (size_t i = 0; i != imageHeight * imageWidth; ++i) {
        observation[i] = *(dataPtr);
        dataPtr++;
    }
    return observation;
}

void DepthSensor::getObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits) const
{
    gazebo::sensors::DepthCameraSensor* depthCameraSensor = static_cast<gazebo::sensors::DepthCameraSensor*>(sensor_.get());
    unsigned int imageHeight = depthCameraSensor->DepthCamera()->ImageHeight();
    unsigned int imageWidth = depthCameraSensor->DepthCamera()->ImageWidth();
    FloatType nearClip = depthCameraSensor->DepthCamera()->NearClip();
    FloatType farClip = depthCameraSensor->DepthCamera()->FarClip();
    lowerLimits = VectorFloat(imageHeight * imageWidth, nearClip);
    upperLimits = VectorFloat(imageHeight * imageWidth, farClip);
}

}
