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

using std::cout;
using std::endl;

namespace oppt
{

/**
 * Virtual base class for a Body
 */
class Body
{
public:
    _NO_COPY_BUT_MOVE(Body)
    Body(const std::string& name, const geometric::Pose &worldPose):
        name_(name),
        worldPose_(worldPose),
        enabled_(true),
        static_(true)
    {

    }

    /**
     * @brief Creates a clone of this body
     *
     * @return A oppt::BodyUniquePtr to the cloned body
     */
    virtual BodyUniquePtr clone() const = 0;

    /**
     * @brief Add a collision geometry to this body
     * @param collisionGeometry The oppt::GeometryUniquePtr collision geometry that is added to the body
     */
    virtual void addCollisionGeometry(GeometryUniquePtr collisionGeometry) = 0;

    /**
     * Update the internal collision object. This must be called after the pose of the body changes in order to
     * reflect these changes in the collision object
     */
    virtual void updateCollisionObjects() = 0;

    /**
     * @brief Add a visual geometry to this body
     * @param collisionGeometry The oppt::GeometryUniquePtr visual geometry that is added to the body
     */
    virtual void addVisualGeometry(GeometryUniquePtr visualGeometry) = 0;

    /**
     * @brief Performs a collision check of this body with the oppt::CollisionObject provided in collisionRequest
     * @param collisionRequest A pointer to a oppt::CollisionRequest
     * @return A oppt::CollisionReportSharedPtr which contains collision information
     */
    CollisionReportSharedPtr collides(const CollisionRequest *collisionRequest) const {
        CollisionReportSharedPtr collisionReport(new CollisionReport);
        for (auto &collisionObject : opptCollisionObjects_) {
            CollisionReportSharedPtr cp = collisionObject->collides(collisionRequest);
            if (cp->collides)
                collisionReport->collides;
            for (size_t i = 0; i != cp->collisionPairs.size(); ++i) {
                collisionReport->collisionPairs.push_back(cp->collisionPairs[i]);
            }
            for (size_t i = 0; i != cp->contacts.size(); ++i) {
                collisionReport->contacts.push_back(std::move(cp->contacts[i]));
            }
        }

        return collisionReport;
    }

    /**
     * @brief Set the world pose of this Body
     * @param pose The new geometric::Pose of the body
     */
    void setWorldPose(const geometric::Pose &pose) {
        for (auto &opptCollisionObject : opptCollisionObjects_) {
            geometric::Pose relativeCollisionPose = opptCollisionObject->getCollisionGeometry()->getWorldPose() - worldPose_;
            opptCollisionObject->getCollisionGeometry()->setWorldPose(relativeCollisionPose + pose);
        }

        for (auto &visualGeometry : visualGeometries_) {
            geometric::Pose relativeVisualPose = visualGeometry->getWorldPose() - worldPose_;
            visualGeometry->setWorldPose(relativeVisualPose + pose);
        }

        worldPose_ = pose;
    }

    void setEnabled(const bool& enabled)
    {
        enabled_ = enabled;
    }

    bool isEnabled() const
    {
        return enabled_;
    }

    void setStatic(const bool& isStatic)
    {
        static_ = isStatic;
    }

    bool isStatic() const
    {
        return static_;
    }

    void serializeToSDF(std::ostream& os) const
    {
        std::string sdfString = toSDFString();
        os << sdfString;
    }

    virtual std::string toSDFString() const = 0;

    /**
     * @brief Get the vector of oppt::OpptCollisionObjects for this body
     */
    std::vector<OpptCollisionObject*> getOpptCollisionObjects() const
    {
        std::vector<OpptCollisionObject*> collisionObjects;
        collisionObjects.reserve(opptCollisionObjects_.size());
        for (auto &collisionObject : opptCollisionObjects_) {
            collisionObjects.push_back(collisionObject.get());
        }

        return collisionObjects;
    }

    std::string getName() const {
        return name_;
    }

    VectorGeometryPtr getVisualGeometries() const {
        VectorGeometryPtr visualGeometries(visualGeometries_.size(), nullptr);
        for (size_t i = 0; i != visualGeometries_.size(); ++i) {
            visualGeometries[i] = visualGeometries_[i].get();
        }

        return visualGeometries;
    }

    void setColor(const VectorFloat& color)
    {
        for (auto &visualGeometry : visualGeometries_) {
            visualGeometry->setColor(color);
        }
    }

    VectorFloat getColor() const
    {
        if (visualGeometries_.size() != 0) {
            return visualGeometries_[0]->getColor();
        }

        return VectorFloat({0.0, 0.0, 0.0, 1.0});
    }

    /**
     * @brief Get the world pose of this oppt::Body
     *
     * @return The world pose of this Body
     */
    geometric::Pose getWorldPose() const {
        return worldPose_;
    }

protected:
    std::string name_;

    bool enabled_;

    bool static_;

    geometric::Pose worldPose_;

    std::vector<GeometryUniquePtr> visualGeometries_;

    std::vector<OpptCollisionObjectUniquePtr> opptCollisionObjects_;

};

/// @brief Collision data stores the collision request and the result given by collision algorithm.
struct CollisionData {
    _NO_COPY_BUT_MOVE(CollisionData)
    CollisionData()
    {
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
