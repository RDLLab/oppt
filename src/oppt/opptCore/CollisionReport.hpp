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
#ifndef __OPPT_COLLISION_REPORT_HPP__
#define __OPPT_COLLISION_REPORT_HPP__
#include "typedefs.hpp"
#include "CollisionObject.hpp"

namespace oppt
{

/**
 * Datastructure for collision queries
 */
class CollisionReport
{
public:
    _NO_COPY_BUT_MOVE(CollisionReport)
        
    CollisionReport() = default;

    virtual ~CollisionReport() = default;

    /**
     * @brief Boolean flag that determines if a collision was detected
     */
    bool collides = false;    

    /**
     * @brief vector of pairs colliding bodies
     */
    std::vector<std::pair<std::string, std::string>> collisionPairs;

    /**
     * @brief Vector of oppt::ContactUniquePtr. If enableContact is false, this vector will be empty
     */
    VectorContacts contacts;

    /**
     * @brief Time (between 0 and 1) of contact. Will be populated in a continuous collision request 
     */
    FloatType timeOfContact = 0.0;

    
};

}

#endif
