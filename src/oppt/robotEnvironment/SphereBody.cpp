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
#include "include/SphereBody.hpp"
#include "oppt/opptCore/geometric/Sphere.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using namespace fcl;

namespace oppt
{

SphereBody::SphereBody(const std::string &name,
                       const geometric::Pose& worldPose,
                       const FloatType &radius):
  BodyImpl(name, worldPose),
  radius_(radius)
{
  collisionGeometry_ = std::make_shared<geometric::Sphere>(name, radius, worldPose);
  createCollisionObject();
}

BodyUniquePtr SphereBody::clone() const
{
  VectorFloat color = getColor();
  BodyUniquePtr clonedBody = std::make_unique<SphereBody>(name_,
                             worldPose_,
                             radius_);
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