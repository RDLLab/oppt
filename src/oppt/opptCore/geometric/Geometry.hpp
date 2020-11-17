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
#ifndef __OPPT_GEOMETRY_HPP__
#define __OPPT_GEOMETRY_HPP__
#include "oppt/opptCore/typedefs.hpp"
#include "oppt/opptCore/constants.hpp"
#include "Pose.hpp"

namespace oppt
{

/**
* @namespace oppt::geometric
* @brief Namespace for geometries
*/
namespace geometric
{

/**
 * Base class for geometries
 */
class Geometry
{
public:
    _NO_COPY_BUT_MOVE(Geometry)

    Geometry() = delete;

    virtual ~Geometry() = default;

    _STATIC_CAST

    /**
     * @brief Get the name of the geometry
     */
    std::string getName() const;

    /**
     * @brief Set the name of the geometry
     */
    void setName(const std::string& name);

    /**
     * @brief Set the world pose of the geometry
     */
    void setWorldPose(const Pose& pose);

    /**
     * @brief Get the world pose of the geometry
     */
    Pose getWorldPose() const;

    /**
     * @brief Copy the geometry
     * @return A shared_ptr to the copied geometry
     */
    virtual GeometryUniquePtr copy() const;

    /**
     * @brief Copy the geometry without constructing the collision object
     * @return A shared_ptr to the copied geometry
     */
    virtual GeometryUniquePtr shallowCopy() const = 0;

    /**
     * @brief Create the underlying oppt::FCLCollisionGeometrySharedPtr
     */
    virtual void createCollisionGeometry() = 0;

    /**
     * @brief Set the color of the geometry
     * @param color a oppt::VectorFloat encoding the RGBD color
     */
    void setColor(const oppt::VectorFloat& color);

    /**
     * @brief Get the color of the geometry
     * @param A oppt::VectorFloat encoding the RGBD color
     */
    oppt::VectorFloat getColor() const;

    /**
     * @brief Get the GeometryType of this geometry
     */
    GeometryType getType() const;

    /**
     * @brief Get a pointer to the oppt::FCLCollisionGeometry for this geometry
     */
    FCLCollisionGeometrySharedPtr getCollisionGeometry() const;

    /**
     * @brief Get the SDF description of this geometry
     * @return A std::string containting the SDF description of this geometry
     */
    virtual std::string toSDFString() const = 0;

protected:
    std::string name_;

    Pose worldPose_;

    oppt::VectorFloat color_;

    const unsigned int type_;

    FCLCollisionGeometrySharedPtr collisionGeometry_ = nullptr;

    Geometry(const GeometryType& type, const std::string& name, const Pose &worldPose);

    //Geometry(const GeometryType& type);
};
}
}

#endif
