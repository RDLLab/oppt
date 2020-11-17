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
 * A data structure which is added to the underlying fcl::CollisionObject
 */
struct OpptCollisionObjectData {
    OpptCollisionObject *opptCollisionObject = nullptr;
};

/**
* Wrapper class around a oppt::CollisionObjectSharedPtr
*/
class OpptCollisionObject
{
public:
    _NO_COPY_BUT_MOVE(OpptCollisionObject)
    

    /**
     * @brief Construct from a oppt::GeometryUniquePtr
     */
    OpptCollisionObject(GeometryUniquePtr collisionGeometry);

    /**
     * @brief Get the underlying oppt::FCLCollisionObject
     */
    FCLCollisionObject * getFCLCollisionObject() const;

    /**
     * @brief Get a pointer to the underlying collision geometry
     */
    geometric::Geometry *getCollisionGeometry() const;
    
    /**
     * @brief Performs a collision check based on the provided oppt::CollisionRequest
     * @param collisionRequest Pointer to a collisionRequest
     * @return A oppt::CollisionReportSharedPtr containing collision information
     */
    CollisionReportSharedPtr collides(const CollisionRequest *collisionRequest) const;

private:
    GeometryUniquePtr collisionGeometry_ = nullptr;

    FCLCollisionObjectUniquePtr collisionObject_ = nullptr;    

    std::unique_ptr<OpptCollisionObjectData> collisionObjectData_ = nullptr;
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