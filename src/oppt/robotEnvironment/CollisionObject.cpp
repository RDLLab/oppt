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

using std::cout;
using std::endl;

namespace oppt
{

OpptCollisionObject::OpptCollisionObject(FCLCollisionObjectUniquePtr collisionObject, const std::string& name):
    collisionObject_(std::move(collisionObject)), 
    name_(name)
{

}

FCLCollisionObject *OpptCollisionObject::getCollisionObject() const
{
    return collisionObject_.get();
}

std::string OpptCollisionObject::getName() const
{
    return name_;
}

CollisionReportSharedPtr OpptCollisionObject::collides(const CollisionRequest *collisionRequest) const
{
    bool collides = false;
    CollisionReportSharedPtr collisionReport(new DiscreteCollisionReport());
    collisionReport->collides = false;
    for (size_t i = 0; i != collisionRequest->collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        request.enable_contact = collisionRequest->enableContact;
        request.num_max_contacts = collisionRequest->maxNumContacts;
        fcl::collide(collisionObject_.get(),
                     collisionRequest->collisionObjects[i]->getCollisionObject(),
                     request,
                     result);
        if (result.isCollision()) {
            collisionReport->collides = true;
            auto dC = static_cast<DiscreteCollisionReport *>(collisionReport.get());
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
                    dC->contacts.push_back(std::move(contact));
                }
            }            
        }
    }

    return collisionReport;
}

}
