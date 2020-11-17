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
#include "oppt/opptCore/geometric/Sphere.hpp"

namespace oppt
{
namespace geometric
{
Sphere::Sphere(const std::string& name,
               const FloatType& radius,
               const Pose& worldPose):
    Geometry(SPHERE, name, worldPose),
    radius_(radius) {
    
}

GeometryUniquePtr Sphere::shallowCopy() const {
    std::string name = name_;
    FloatType radius = radius_;
    Pose worldPose(worldPose_);    
    GeometryUniquePtr copiedGeometry(new Sphere(name, radius, worldPose));
    copiedGeometry->setColor(getColor());
    return std::move(copiedGeometry);    
}

FloatType Sphere::getRadius() const {
    return radius_;
}

std::string Sphere::toSDFString() const {
    std::string serString = "";
    serString += "<geometry>";
    serString += "<sphere>";
    serString += "<radius>";
    serString += std::to_string(radius_);
    serString += "</radius>";
    serString += "</sphere>";
    serString += "</geometry>";
    return serString;
}

void Sphere::createCollisionGeometry() {
    collisionGeometry_ = FCLCollisionGeometrySharedPtr(new fcl::Sphere(radius_));
}

}
}
