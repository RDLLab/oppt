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
#include "oppt/opptCore/geometric/Cylinder.hpp"

namespace oppt
{
namespace geometric
{
Cylinder::Cylinder(const std::string& name,
                   const FloatType& radius,
                   const FloatType& length,
                   const Pose& worldPose):
    Geometry(CYLINDER, name, worldPose),
    radius_(radius),
    length_(length) {    
    
}

GeometryUniquePtr Cylinder::shallowCopy() const {
    std::string name = name_;
    FloatType radius = radius_;
    FloatType length = length_;    
    Pose worldPose(worldPose_);    
    GeometryUniquePtr copiedGeometry(new Cylinder(name, radius, length, worldPose));
    copiedGeometry->setColor(getColor());
    return std::move(copiedGeometry);    
}

FloatType Cylinder::getRadius() const {
    return radius_;
}

FloatType Cylinder::getLength() const {
    return length_;
}

std::string Cylinder::toSDFString() const {
    std::string serString = "";
    serString += "<geometry>";
    serString += "<cylinder>";
    serString += "<radius>";
    serString += std::to_string(radius_);
    serString += "</radius>";
    serString += "<length>";
    serString += std::to_string(length_);
    serString += "</length>";
    serString += "</cylinder>";
    serString += "</geometry>";
    return serString;
}

void Cylinder::createCollisionGeometry() {
    collisionGeometry_ = FCLCollisionGeometrySharedPtr(new fcl::Cylinder(radius_, length_));
}

}
}
