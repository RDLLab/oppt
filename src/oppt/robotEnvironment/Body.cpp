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

    if (cdata_->done) {
        cout << "DONE" << endl;
        return true;
    }

    size_t currentNumContacts = result.numContacts();
    fcl::collide(o1, o2, request, result);
    if (result.isCollision()) {
        cdata_->o1.push_back(o1);
        cdata_->o2.push_back(o2);
    }

    if (result.numContacts() > currentNumContacts) {
        //cdata_->o1.push_back(o1);
        //cdata_->o2.push_back(o2);
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

BodyUniquePtr BodyImpl::clone() const {
    geometric::Pose worldPose(worldPose_);
    BodyUniquePtr clonedBody(new BodyImpl(getName(), worldPose));
    for (size_t i = 0; i != visualGeometries_.size(); ++i) {
        clonedBody->addVisualGeometry(std::move(visualGeometries_[i]->copy()));
    }

    for (size_t i = 0; i != opptCollisionObjects_.size(); ++i) {
        clonedBody->addCollisionGeometry(std::move(opptCollisionObjects_[i]->getCollisionGeometry()->copy()));
    }

    clonedBody->updateCollisionObjects();
    clonedBody->setEnabled(isEnabled());
    clonedBody->setStatic(isStatic());
    return std::move(clonedBody);    
}

void BodyImpl::addCollisionGeometry(GeometryUniquePtr collisionGeometry) {
    if (!collisionGeometry)
        return;
    OpptCollisionObjectUniquePtr opptCollisionObject =
        OpptCollisionObjectUniquePtr(new OpptCollisionObject(std::move(collisionGeometry)));
    opptCollisionObjects_.push_back(std::move(opptCollisionObject));    
}

void BodyImpl::updateCollisionObjects() {
    for (auto &collisionObject : opptCollisionObjects_) {
        geometric::Pose pose = collisionObject->getCollisionGeometry()->getWorldPose();
        fcl::Quaternion3f q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
        fcl::Vec3f trans_vec(pose.position.x(), pose.position.y(), pose.position.z());
        collisionObject->getFCLCollisionObject()->setTransform(q, trans_vec);
        collisionObject->getFCLCollisionObject()->computeAABB();
    }    
}

void BodyImpl::addVisualGeometry(GeometryUniquePtr visualGeometry) {
    visualGeometries_.push_back(std::move(visualGeometry));
}

std::string BodyImpl::toSDFString() const
{
    std::string serString = "";
    geometric::Pose pose = getWorldPose();
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
    for (auto &collisionObject: opptCollisionObjects_) {
        serString += "<collision name='";        
        serString += name_ + "_" + collisionObject->getCollisionGeometry()->getName();
        serString += "_collision'>";
        serString += collisionObject->getCollisionGeometry()->toSDFString();
        serString += "</collision>";
    }

    for (auto &visualGeometry: visualGeometries_) {
        serString += "<visual name='";
        serString += name_ + "_" + visualGeometry->getName();
        serString += "_visual'>";
        serString += visualGeometry->toSDFString();
        serString += "</visual>";
    }
    
    serString += "</link>";
    serString += "</model>";    
    return serString;
}

}