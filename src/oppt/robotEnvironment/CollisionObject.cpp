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
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#include "oppt/opptCore/CollisionReport.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"

using std::cout;
using std::endl;

namespace oppt
{

OpptCollisionObject::OpptCollisionObject(GeometryUniquePtr collisionGeometry):
    collisionGeometry_(std::move(collisionGeometry)),
    collisionObjectData_(new OpptCollisionObjectData) {
    geometric::Pose pose = collisionGeometry_->getWorldPose();
    fcl::Quaternion3f q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
    fcl::Vec3f trans_vec(pose.position[0], pose.position[1], pose.position[2]);
    fcl::Transform3f transform(q, trans_vec);
    if (collisionGeometry_->getCollisionGeometry() == nullptr)
        collisionGeometry_->createCollisionGeometry();
    collisionObject_ = FCLCollisionObjectUniquePtr(new fcl::CollisionObject(collisionGeometry_->getCollisionGeometry(), transform));
    collisionObjectData_->opptCollisionObject = this;
    collisionObject_->setUserData((void*)(collisionObjectData_.get()));
}

FCLCollisionObject *OpptCollisionObject::getFCLCollisionObject() const
{
    return collisionObject_.get();
}

geometric::Geometry *OpptCollisionObject::getCollisionGeometry() const {
    return collisionGeometry_.get();
}

CollisionReportSharedPtr OpptCollisionObject::collides(const CollisionRequest *collisionRequest) const
{
    bool collides = false;
    CollisionReportSharedPtr collisionReport(new CollisionReport());
    collisionReport->collisionPairs.reserve(collisionRequest->collisionObjects.size());
    collisionReport->collides = false;
    for (size_t i = 0; i != collisionRequest->collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        request.enable_contact = collisionRequest->enableContact;
        request.num_max_contacts = collisionRequest->maxNumContacts;
        fcl::collide(collisionObject_.get(),
                     collisionRequest->collisionObjects[i]->getFCLCollisionObject(),
                     request,
                     result);
        if (result.isCollision()) {
            auto collisionPair =
                std::make_pair(collisionGeometry_->getName(),
                               collisionRequest->collisionObjects[i]->getCollisionGeometry()->getName());
            collisionReport->collisionPairs.push_back(collisionPair);
            collisionReport->collides = true;
            if (request.enable_contact) {
                std::vector<fcl::Contact> contacts;
                result.getContacts(contacts);
                for (size_t j = 0; j != contacts.size(); ++j) {
                    ContactUniquePtr contact(new Contact);
                    contact->contactPosition =
                        VectorFloat({contacts[j].pos[0], contacts[j].pos[1], contacts[j].pos[2]});
                    contact->contactNormal =
                        VectorFloat({contacts[j].normal[0], contacts[j].normal[1], contacts[j].normal[2]});
                    contact->penetrationDepth = contacts[j].penetration_depth;
                    contact->body1 = this;
                    contact->body2 = collisionRequest->collisionObjects[i];
                    collisionReport->contacts.push_back(std::move(contact));
                }
            }
        }
    }

    return collisionReport;
}

}
