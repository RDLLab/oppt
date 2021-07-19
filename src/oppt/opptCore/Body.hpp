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
#ifndef __OPPT_BODY_HPP__
#define __OPPT_BODY_HPP__
#include "includes.hpp"
#include "typedefs.hpp"
#include "CollisionReport.hpp"
#include "logging.hpp"
#include "MathUtils.hpp"
#include "geometric/Geometry.hpp"

namespace oppt
{

/**
 * Virtual base class for a Body
 */
class Body
{
public:
    friend class SDFEnvironmentParser;
    _NO_COPY_BUT_MOVE(Body)
    Body(const std::string& name, const geometric::Pose &worldPose);

    /**
     * @brief Creates a clone of this body
     *
     * @return A oppt::BodyUniquePtr to the cloned body
     */
    virtual BodyUniquePtr clone() const;

    /**
     * @brief Add a collision geometry to this body
     * @param collisionGeometry The oppt::GeometryUniquePtr collision geometry that is added to the body
     */
    virtual void addCollisionGeometry(GeometryUniquePtr collisionGeometry);

    /**
     * Update the internal collision object. This must be called after the pose of the body changes in order to
     * reflect these changes in the collision object
     */
    virtual void updateCollisionObjects();

    /**
     * @brief Add a visual geometry to this body
     * @param collisionGeometry The oppt::GeometryUniquePtr visual geometry that is added to the body
     */
    virtual void addVisualGeometry(GeometryUniquePtr visualGeometry);

    /**
     * @brief Performs a collision check of this body with the oppt::CollisionObject provided in collisionRequest
     * @param collisionRequest A pointer to a oppt::CollisionRequest
     * @return A oppt::CollisionReportSharedPtr which contains collision information
     */
    CollisionReportSharedPtr collides(const CollisionRequest *collisionRequest) const;

    /**
     * @brief Set the world pose of this Body
     * @param pose The new geometric::Pose of the body
     */
    void setWorldPose(const geometric::Pose &pose);

    void setEnabled(const bool& enabled);

    bool isEnabled() const;

    void setStatic(const bool& isStatic);

    bool isStatic() const;

    void serializeToSDF(std::ostream& os) const;

    /**
     * @Brief Converts this body into a SDF string
     */
    virtual std::string toSDFString() const;

    /**
     * @brief Get the vector of oppt::OpptCollisionObjects for this body
     */
    std::vector<OpptCollisionObject*> getOpptCollisionObjects() const;

    /**
     * @brief Get the name of this body
     */
    std::string getName() const;

    /**
     * @brief Get the scoped name of this body
     */
    std::string getScopedName() const;

    /**
     * @brief Get a vector of oppt::VectorGeometryPtr containing the visual geometries of this body
     */
    VectorGeometryPtr getVisualGeometries() const;

    void setColor(const VectorFloat& color);

    VectorFloat getColor() const;

    /**
     * @brief Get the world pose of this oppt::Body
     *
     * @return The world pose of this Body
     */
    geometric::Pose getWorldPose() const;

protected:
    std::string name_;

    std::string scopedName_;

    bool enabled_;

    bool static_;

    geometric::Pose worldPose_;

    std::vector<GeometryUniquePtr> visualGeometries_;

    std::vector<OpptCollisionObjectUniquePtr> opptCollisionObjects_;

};

/// @brief Collision data stores the collision request and the result given by collision algorithm.
struct CollisionData {
    _NO_COPY_BUT_MOVE(CollisionData)
    CollisionData() {
        done = false;
    }

    virtual ~CollisionData() = default;

    /// @brief Collision request
    fcl::CollisionRequest request;

    /// @brief Collision result
    fcl::CollisionResult result;

    /// @brief Whether the collision iteration can stop
    bool done;

    std::vector<fcl::CollisionObject*> o1;

    std::vector<fcl::CollisionObject*> o2;
};

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata);

}

#endif
