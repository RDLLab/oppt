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

namespace oppt {
namespace geometric {
std::string Geometry::getName() const {
	return name_;
}

void Geometry::setName(const std::string& name) {
	name_ = name;
}

void Geometry::setWorldPose(const Pose& pose) {
	worldPose_ = pose;
}

Pose Geometry::getWorldPose() const {
	return worldPose_;
}

GeometryUniquePtr Geometry::copy() const {
	GeometryUniquePtr copiedGeometry = std::move(shallowCopy());
	copiedGeometry->createCollisionGeometry();
	return std::move(copiedGeometry);
}

void Geometry::setColor(const oppt::VectorFloat& color) {
	color_ = color;
}

oppt::VectorFloat Geometry::getColor() const {
	return color_;
}

GeometryType Geometry::getType() const {
	return type_;
}

FCLCollisionGeometrySharedPtr Geometry::getCollisionGeometry() const {
	return collisionGeometry_;
}

Geometry::Geometry(const GeometryType& type, const std::string& name, const Pose &worldPose):
	type_(type),
	name_(name),
	worldPose_(worldPose),
	color_( {0.5, 0.5, 0.5, 1.0}),
collisionGeometry_(nullptr) {

}

/**Geometry::Geometry(const GeometryType& type):
	type_(type),
	name_(""),
	worldPose_(),
	color_( {0.5, 0.5, 0.5, 1.0}) {

}*/

} // end namespace geometric

} // end namespace oppt