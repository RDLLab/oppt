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
	CollisionRequest() = default;

	virtual ~CollisionRequest() = default;

	_STATIC_CAST
	
	_NO_COPY_BUT_MOVE(CollisionRequest)

	/**
	 * @brief Contains the oppt::CollisionObjects for which collision checking is performed
	 */
	VectorCollisionObjectPtr collisionObjects;

	/**
	 * @brief Collect contact information during the collision check
	 */
	bool enableContact = false;

	/**
	 * @brief Maximum number of contacts
	 */
	unsigned int maxNumContacts = 1;

	/**
	 * @brief Determines if a continuous collision check is performed
	 */
	bool continuousCollisionCheck = false;

	/**
	 * @brief Number of iterations for the continuous collision check (only used when continuousCollisionCheck is true)
	 */
	unsigned int numIterations = 10;

	/**
	 * @brief Contains the goal transformations of the collision objects provided in collisionObjects (only used when continuousCollisionCheck is true)
	 */
	VectorFCLTransform3f tfGoal;
};

/**
 * @brief A std::shared_ptr to oppt::CollisionRequest
 */
typedef std::shared_ptr<CollisionRequest> CollisionRequestSharedPtr;

}

#endif