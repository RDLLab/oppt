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
#include "oppt/opptCore/geometric/Box.hpp"

namespace oppt
{
namespace geometric
{
Box::Box(const std::string& name,
         const oppt::VectorFloat& dimensions,
         const Pose& worldPose):
    Geometry(BOX, name, worldPose),
    dimensions_(dimensions) {    
    
}

VectorFloat Box::getDimensions() const {
    return dimensions_;
}

GeometryUniquePtr Box::shallowCopy() const {
    std::string name = name_;
    Pose worldPose(worldPose_);    
    VectorFloat dimensions = dimensions_;
    GeometryUniquePtr copiedGeometry(new Box(name, dimensions, worldPose));    
    copiedGeometry->setColor(getColor());
    return std::move(copiedGeometry);    
}

std::string Box::toSDFString() const {
    std::string serString = "";
    serString += "<geometry>";
    serString += "<box>";
    serString += "<size>";
    serString += std::to_string(dimensions_[0]) + " ";
    serString += std::to_string(dimensions_[1]) + " ";
    serString += std::to_string(dimensions_[2]) + " ";
    serString += "</size>";
    serString += "</box>";
    serString += "</geometry>";
    return serString;
}

void Box::createCollisionGeometry() {
    collisionGeometry_ = FCLCollisionGeometrySharedPtr(new fcl::Box(dimensions_[0], dimensions_[1], dimensions_[2]));
}

}
}
