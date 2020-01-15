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
#include "include/CylinderBody.hpp"
#include "oppt/opptCore/geometric/Cylinder.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{

CylinderBody::CylinderBody(const std::string &name,
                           const geometric::Pose &worldPose,
                           const FloatType& radius,
                           const FloatType& length):
  BodyImpl(name, worldPose)
{
  collisionGeometry_ = std::make_shared<geometric::Cylinder>(name, radius, length, worldPose);
  createCollisionObject();
}

BodyUniquePtr CylinderBody::clone() const
{
  FloatType radius = static_cast<geometric::Cylinder*>(collisionGeometry_.get())->getRadius();
  FloatType length = static_cast<geometric::Cylinder*>(collisionGeometry_.get())->getRadius();
  VectorFloat color = getColor();
  BodyUniquePtr clonedBody = std::make_unique<CylinderBody>(name_,
                             worldPose_,
                             radius,
                             length);
  clonedBody->setStatic(static_);
  clonedBody->setEnabled(enabled_);
  clonedBody->setColor(color);
  auto clonedCollisionGeometry = clonedBody->getCollisionGeometry();
  clonedCollisionGeometry->setWorldPose(getCollisionGeometry()->getWorldPose());
  if (visualGeometry_) {
    auto clonedVisualGeometry = visualGeometry_->copy();
    clonedBody->initVisualGeometry(clonedVisualGeometry);
  }
  return std::move(clonedBody);
}

}