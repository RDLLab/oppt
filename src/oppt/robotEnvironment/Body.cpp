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
#include "include/Body.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata)
{
    CollisionData* cdata_ = static_cast<CollisionData*>(cdata);
    const fcl::CollisionRequest& request = cdata_->request;
    fcl::CollisionResult& result = cdata_->result;

    if (cdata_->done) return true;
    size_t currentNumContacts = result.numContacts();

    fcl::collide(o1, o2, request, result);

    if (result.numContacts() > currentNumContacts) {
        cdata_->o1.push_back(o1);
        cdata_->o2.push_back(o2);
        if (!(request.enable_contact)) {
            cdata_->done = true;
        } else {
            if (!request.enable_cost && (result.numContacts() >= request.num_max_contacts)) {
                cdata_->done = true;
            }
        }
    }

    return cdata_->done;
}

BodyImpl::BodyImpl(const std::string& name, const geometric::Pose &worldPose):
    oppt::Body(name, worldPose)
{

}

void BodyImpl::createCollisionObject() {
    geometric::Pose pose = collisionGeometry_->getWorldPose();
    fcl::Quaternion3f q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
    fcl::Vec3f trans_vec(pose.position[0], pose.position[1], pose.position[2]);
    fcl::Transform3f transform(q, trans_vec);
    //CollisionGeometrySharedPtr collisionGeometry = collisionGeometry_->getCollisionGeometry();
    FCLCollisionObjectUniquePtr collisionObject(new fcl::CollisionObject(collisionGeometry_->getCollisionGeometry(), transform));
    opptCollisionObject_ = OpptCollisionObjectUniquePtr(new OpptCollisionObject(std::move(collisionObject), getName()));
}


void BodyImpl::updateCollisionObject() {
    if (opptCollisionObject_ && collisionGeometry_) {
        geometric::Pose pose = collisionGeometry_->getWorldPose();
        fcl::Quaternion3f q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
        fcl::Vec3f trans_vec(pose.position.x(), pose.position.y(), pose.position.z());
        opptCollisionObject_->getCollisionObject()->setTransform(q, trans_vec);
        opptCollisionObject_->getCollisionObject()->computeAABB();
    }
}

void BodyImpl::initVisualGeometry(const GeometrySharedPtr &visualGeometry) {
    visualGeometry_ = visualGeometry;
}


std::string BodyImpl::toSDFString() const
{
    std::string serString = "";
    geometric::Pose pose = collisionGeometry_->getWorldPose();
    Vector3f eulerAngles = math::quaternionToEulerAngles(pose.orientation);

    //serString += "<sdf version='1.6'>";
    serString += "<model name=\"" + name_ + "\">";
    serString += "<pose frame=''>";
    serString += std::to_string(pose.position.x()) + " ";
    serString += std::to_string(pose.position.y()) + " ";
    serString += std::to_string(pose.position.z()) + " ";
    serString += std::to_string(eulerAngles[0]) + " ";
    serString += std::to_string(eulerAngles[1]) + " ";
    serString += std::to_string(eulerAngles[2]);
    serString += "</pose>";
    serString += "<static>";
    if (static_) {
        serString += "1";
    } else {
        serString += "0";
    }
    serString += "</static>";
    serString += "<link name=\"" + name_ + "_link\">";
    if (enabled_) {
        serString += "<collision name='";
        serString += name_ + "_collision'>";
        serString += collisionGeometry_->toSDFString();
        serString += "</collision>";
    }

    if (visualGeometry_) {
        serString += "<visual name='";
        serString += name_ + "_visual'>";
        serString += visualGeometry_->toSDFString();
        serString += "</visual>";
    }
    serString += "</link>";
    serString += "</model>";
    //serString += "</sdf>";
    return serString;
}

/**bool BodyImpl::collides(const std::vector<oppt::CollisionObjectSharedPtr>& collisionObjects,
                            unsigned int& collidingBodyIndex) const
{
    for (size_t i = 0; i != collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(collisionObjects[i].get(),
                     opptCollisionObject_->getCollisionObject().get(),
                     request,
                     result);
        if (result.isCollision()) {
            collidingBodyIndex = i;
            return true;
        }
    }

    return false;
}

bool BodyImpl::collides(const std::vector<oppt::BodySharedPtr>& otherBodies) const
{
    std::vector<oppt::CollisionObjectSharedPtr> collisionObjects(otherBodies.size(), nullptr);
    for (size_t i = 0; i != otherBodies.size(); ++i) {
        collisionObjects[i] = otherBodies[i]->getOpptCollisionObject()->getCollisionObject();
    }

    unsigned int bodyIdx = 0;
    return collides(collisionObjects, bodyIdx);
}

bool BodyImpl::collides(const VectorFloat& point) const
{
    Vec3f p_vec(point[0], point[1], point[2]);
    return opptCollisionObject_->getCollisionObject()->getAABB().contain(p_vec);
}*/

/**FloatType BodyImpl::distance(const std::vector<oppt::CollisionObjectSharedPtr>& collisionObjects) const
{
    FloatType min_distance = std::numeric_limits<FloatType>::infinity();
    for (size_t i = 0; i != collisionObjects.size(); i++) {
        fcl::DistanceRequest request;
        fcl::DistanceResult result;
        fcl::distance(collisionObjects[i].get(),
                      opptCollisionObject_->getCollisionObject().get(),
                      request,
                      result);
        if (result.min_distance < min_distance)
            min_distance = result.min_distance;

    }

    return min_distance;
}*/

/**bool BodyImpl::collidesContinuous(const oppt::CollisionObjectSharedPtr& collisionObjectStart,
                                      const oppt::CollisionObjectSharedPtr& collisionObjectGoal) const
{
    fcl::ContinuousCollisionRequest request(20,
                                            0.0001,
                                            CCDM_LINEAR,
                                            GST_LIBCCD,
                                            CCDC_NAIVE);
    fcl::ContinuousCollisionResult result;
    fcl::continuousCollide(collisionObjectStart.get(),
                           collisionObjectGoal->getTransform(),
                           opptCollisionObject_->getCollisionObject().get(),
                           opptCollisionObject_->getCollisionObject()->getTransform(),
                           request,
                           result);
    return result.is_collide;
}*/

}
