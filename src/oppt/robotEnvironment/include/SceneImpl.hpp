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
#ifndef _SCENE_IMPL_HPP_
#define _SCENE_IMPL_HPP_
#include "oppt/opptCore/Scene.hpp"

namespace oppt
{
/**
* Implementation of the geometric representation of the environment
* inside the oppt::RobotEnvironment
*/
class SceneImpl: public Scene
{
public:
    /** @brief Default constructor */
    SceneImpl();

    /** @brief Default constructor */
    virtual ~SceneImpl();

    virtual SceneSharedPtr clone() const override;

    virtual void addBody(BodyUniquePtr body, bool executeCallback = true) override;

    virtual void addBodies(VectorBodyUniquePtr& bodies, bool executeCallback = true) override;

    virtual void removeModel(const std::string &modelName) override;

    virtual bool removeBody(std::string bodyName, bool executeCallback = true) override;

    virtual bool removeBodies(std::vector<std::string>& body_names, bool executeCallback = true) override;

    virtual bool changeBodyPose(const std::string& name, const geometric::Pose& pose, bool executeCallback = true) override;

    virtual VectorBodyPtr getBodies() const override;

    virtual Body* getBody(std::string name) const override;

    virtual CollisionReportSharedPtr makeDiscreteCollisionReport(const CollisionRequest *collisionRequest) override;

    virtual CollisionReportSharedPtr makeContinuousCollisionReport(const CollisionRequest *collisionRequest) override;

    virtual void setRemoveBodyCallback(std::function<void(std::string)> callbackFunction) override;

    virtual void setAddBodyCallback(std::function<void(oppt::Body *const)> callbackFunction) override;

    virtual void setRobotCollisionObjects(VectorCollisionObjectPtr& robotCollisionObjects) override;

    virtual void setCollisionInvariantRobotCollisionObjects(VectorCollisionObjectPtr &robotCollisionObjects) override;

    virtual VectorCollisionObjectPtr getRobotCollisionObjects() const override;

    virtual VectorCollisionObjectPtr getCollisionInvariantRobotCollisionObjects() const override;

    virtual void setChangeBodyPoseCallback(std::function<void(const std::string&, const geometric::Pose&)> callbackFunction) override;

    virtual std::string toSDFString() const override;

protected:
    virtual bool bodyManaged(const std::string& bodyName) const override;

    std::unordered_map<OpptCollisionObject*, std::string> collisionObjectMap_;

    virtual void changeBodyPoses(const VectorString& names, const std::vector<geometric::Pose>& poses, bool executeCallback = true) override;

};

}

#endif
