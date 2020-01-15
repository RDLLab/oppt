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
#ifndef _OPPT_COLLISION_OBJECT_HPP_
#define _OPPT_COLLISION_OBJECT_HPP_
#include "typedefs.hpp"

namespace oppt {
/**
* Wrapper class around a oppt::CollisionObjectSharedPtr
*/
class OpptCollisionObject
{
public:
    /**
     * @brief Construct from a oppt::CollisionObjectSharedPtr and a name for the collision object
     */
    OpptCollisionObject(FCLCollisionObjectUniquePtr collisionObject, const std::string& name);

    /**
     * @brief Get the underlying oppt::CollisionObjectSharedPtr
     */
    FCLCollisionObject * getCollisionObject() const;

    /**
     * @brief Get the name of this oppt::OpptCollisionObject
     */
    std::string getName() const;

    /**
     * @brief Performs a collision check based on the provided oppt::CollisionRequest
     * @param collisionRequest Pointer to a collisionRequest
     * @return A oppt::CollisionReportSharedPtr containing collision information
     */
    CollisionReportSharedPtr collides(const CollisionRequest *collisionRequest) const;

private:
    FCLCollisionObjectUniquePtr collisionObject_ = nullptr;

    std::string name_;
};

/**
 * Structure that represents a contact
 */
struct Contact {
    VectorFloat contactPosition;

    VectorFloat contactNormal;

    FloatType penetrationDepth = 0.0;

    const OpptCollisionObject *body1 = nullptr;

    const OpptCollisionObject *body2 = nullptr;
};
}

#endif