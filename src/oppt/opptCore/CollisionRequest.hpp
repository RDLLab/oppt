/**
 * Copyright 2018
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
#ifndef __OPPT_COLLISION_REQUEST_HPP__
#define __OPPT_COLLISION_REQUEST_HPP__
#include "typedefs.hpp"

namespace oppt {
struct CollisionRequest {

	VectorCollisionObjectPtr collisionObjects;

	bool enableContact = false;

	unsigned int maxNumContacts = 1;

};

typedef std::shared_ptr<CollisionRequest> CollisionRequestSharedPtr;

}

#endif