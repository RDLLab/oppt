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
#ifndef _SCENE_HPP_
#define _SCENE_HPP_
#include "typedefs.hpp"
#include "geometric/Pose.hpp"

namespace oppt
{
/**
* Virtual base class for a scene, which is a geometric
* representation of the environment the robot operates in
*/
class Scene
{
public:
    friend class RobotEnvironment;
    _NO_COPY_BUT_MOVE(Scene)

    Scene():
        bodies_(),
        robotCollisionObjects_(),
        bodiesInManager_(),
        sceneCollisionManager_(new fcl::DynamicAABBTreeCollisionManager()),
        robotCollisionManager_(new fcl::DynamicAABBTreeCollisionManager()) {
    }

    /** @brief Default destructor */
    virtual ~Scene() = default;

    /**
     * @brief Clone this scene and return a oppt::SceneSharedPtr to the cloned scene
     */
    virtual SceneSharedPtr clone() const = 0;

    /**
     * Adds a body to the scene.
     * @param body A oppt::BodyUniquePtr to the body that is being added to the scene
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setAddBodyCallback is called
     */
    virtual void addBody(BodyUniquePtr body, bool executeCallback = true) = 0;

    /**
     * Adds multiple bodies to the scene.
     * @param bodies A oppt::VectorBodyPtr containing of the bodies that are being added to the scene
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setAddBodyCallback is called
     */
    virtual void addBodies(VectorBodyUniquePtr& bodies, bool executeCallback = true) = 0;

    /**
     * Removes a model from the scene.
     * @param modelName The name of the model to be removed
     */
    virtual void removeModel(const std::string &modelName) = 0;

    /**
     * Removes and body from the scene
     * @param bodyName The name of the body that is being removed. If an body with this name doesn't exist,
     * it is being ignored
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setRemoveBodyCallback is called
     */
    virtual bool removeBody(std::string bodyName, bool executeCallback = true) = 0;

    /**
     * Removes multiple bodies from the scene
     * @param bodyName The names of the bodies that are being removed.
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setRemoveBodyCallback is called
     */
    virtual bool removeBodies(std::vector<std::string>& body_names, bool executeCallback = true) = 0;

    /**
     * Change the pose of an body
     * @param name The name of the body whose pose is being changed
     * @param pose The new pose
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setChangeBodyPoseCallback is called
     */
    virtual bool changeBodyPose(const std::string& name, const geometric::Pose& pose, bool executeCallback = true) = 0;

    /**
     * @brief Get a oppt::VectorBodyPtr containing all bodies in this scene
     */
    virtual VectorBodyPtr getBodies() const = 0;

    /**
     * Get a oppt::BodySharedPtr the body with the given name.
     * If no such body exists, a nullptr is returned
     */
    virtual Body *getBody(std::string name) const  = 0;

    /**
     * Performs a discrete collision check on the scene based on a oppt::CollisionReportSharedPtr
     */
    virtual CollisionReportSharedPtr makeDiscreteCollisionReport(const CollisionRequest *collisionRequest) = 0;

    /**
     * Performs a continuous collision check on the scene based on a oppt::CollisionReportSharedPtr
     */
    virtual CollisionReportSharedPtr makeContinuousCollisionReport(const CollisionRequest *collisionRequest) = 0;

    /**
     * @brief Set the callback function this is being called when an body is removed
     */
    virtual void setRemoveBodyCallback(std::function<void(std::string)> callbackFunction) = 0;

    /**
     * @brief Set the callback function this is being called when an body is added
     */
    virtual void setAddBodyCallback(std::function<void(oppt::Body *const)> callbackFunction) = 0;

    /**
     * @brief Set the callback function this is being called the pose of an body is changed
     */
    virtual void setChangeBodyPoseCallback(std::function<void(const std::string&, const geometric::Pose&)> callbackFunction) = 0;

    /**
     * Set the robot collision objects that are being collision-checked against the bodies in the scene
     * when oppt::Scene::makeDiscreteCollisionReport or oppt::Scene::makeContinuousCollisionReport is called
     */
    virtual void setRobotCollisionObjects(VectorCollisionObjectPtr& robotCollisionObjects) = 0;

    /**
     * Set the robot collision objects that are ingnored during collision checking
     */
    virtual void setCollisionInvariantRobotCollisionObjects(VectorCollisionObjectPtr &robotCollisionObjects) = 0;

    /**
     * @brief Return the robot collision objects managed by this scene
     */
    virtual VectorCollisionObjectPtr getRobotCollisionObjects() const = 0;

    /**
     * Get the robot collision objects that are ingnored during collision checking
     */
    virtual VectorCollisionObjectPtr getCollisionInvariantRobotCollisionObjects() const = 0;

    /**
     * @brief Returns an SDF string of the scene
     */
    virtual std::string toSDFString() const = 0;

protected:
    VectorBodyUniquePtr bodies_;

    VectorCollisionObjectPtr robotCollisionObjects_;

    VectorCollisionObjectPtr collisionInvariantRobotCollisionObjects_;

    std::unordered_map<std::string, Body *> bodiesInManager_;
    //VectorString bodiesInManager_;

    std::function<void(std::string)> removeBodyCallback_ = nullptr;

    std::function<void(oppt::Body *const)> addBodyCallback_ = nullptr;

    std::function<void(const std::string&, const geometric::Pose&)> changePoseCallback_ = nullptr;

    std::shared_ptr<fcl::BroadPhaseCollisionManager> sceneCollisionManager_;

    std::shared_ptr<fcl::BroadPhaseCollisionManager> robotCollisionManager_;

    virtual bool bodyManaged(const std::string& bodyName) const = 0;

    virtual void changeBodyPoses(const VectorString& names, const std::vector<geometric::Pose>& poses, bool executeCallback = true) = 0;
};
}

#endif
