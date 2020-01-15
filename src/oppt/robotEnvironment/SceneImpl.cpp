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
#include "include/SceneImpl.hpp"
#include "include/Body.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

namespace oppt
{

SceneImpl::SceneImpl():
    Scene()
{
    sceneCollisionManager_->clear();
    sceneCollisionManager_->setup();
    robotCollisionManager_->clear();
    robotCollisionManager_->setup();

}

SceneImpl::~SceneImpl()
{
    sceneCollisionManager_->clear();
    robotCollisionManager_->clear();
    sceneCollisionManager_.reset();
    robotCollisionManager_.reset();
}

SceneSharedPtr SceneImpl::clone() const
{
    SceneSharedPtr clonedScene = std::make_shared<SceneImpl>();
    for (auto & body : bodies_) {
        BodyUniquePtr clonedBody = std::move(body->clone());
        clonedScene->addBody(std::move(clonedBody));
    }

    return clonedScene;
}

void SceneImpl::addBody(BodyUniquePtr body, bool executeCallback)
{
    Body* bodyPtr = body.get();
    Body* obst = getBody(body->getName());
    if (obst) {
        removeBody(obst->getName());
    }

    bodies_.push_back(std::move(body));
    if (bodyPtr->isEnabled()) {
        bodiesInManager_.push_back(bodyPtr->getName());
        sceneCollisionManager_->registerObject(bodyPtr->getOpptCollisionObject()->getCollisionObject());
        sceneCollisionManager_->update();
        collisionObjectMap_[bodyPtr->getOpptCollisionObject()] = bodyPtr->getName();
    }

    if (executeCallback) {
        if (addBodyCallback_) {
            addBodyCallback_(bodyPtr);
            return;
        }
    }
}

void SceneImpl::addBodies(VectorBodyUniquePtr& bodies, bool executeCallback)
{
    for (size_t i = 0; i != bodies.size(); ++i) {
        addBody(std::move(bodies[i]), executeCallback);
    }
}

void SceneImpl::removeModel(const std::string &modelName) {
    for (size_t i = bodies_.size(); i--;) {
        VectorString nameElems;        
        split(bodies_[i]->getName(), "::", nameElems);
        if (nameElems[0] == modelName) {            
            removeBody(bodies_[i]->getName(), false);
        }
    }    
}

bool SceneImpl::removeBody(std::string bodyName, bool executeCallback)
{
    Body* body = getBody(bodyName);
    if (!body) {
        WARNING("Can't remove body with name '" + bodyName + "'");
        return false;
    }
    for (size_t i = 0; i < bodies_.size(); i++) {
        bool found = false;
        std::string obstName = bodies_[i]->getName();
        if (bodies_[i]->getName() == bodyName) {
            found = true;
        } else if (obstName.find("_nestLink_") != std::string::npos && obstName.find(bodyName) != std::string::npos) {
            found = true;
        }

        if (found) {
            int managedIndex = bodyManaged(body->getName());
            if (managedIndex > -1) {
                sceneCollisionManager_->unregisterObject(bodies_[i]->getOpptCollisionObject()->getCollisionObject());
                sceneCollisionManager_->update();
                bodiesInManager_.erase(bodiesInManager_.begin() + managedIndex);
            }

            if (collisionObjectMap_.find(bodies_[i]->getOpptCollisionObject()) != collisionObjectMap_.end())
                collisionObjectMap_.erase(bodies_[i]->getOpptCollisionObject());
            bodies_.erase(bodies_.begin() + i);
            if (executeCallback) {
                if (removeBodyCallback_) {
                    removeBodyCallback_(bodyName);
                    return true;
                }
            }

            break;
        }
    }

    return true;
}

bool SceneImpl::removeBodies(std::vector<std::string>& body_names, bool executeCallback)
{
    for (auto & k : body_names) {
        removeBody(k, executeCallback);
    }

    return true;
}

bool SceneImpl::changeBodyPose(const std::string& name, const geometric::Pose& pose, bool executeCallback)
{
    Body* body = getBody(name);
    if (!body) {
        WARNING("Can't change pose of body '" + name + "'");
    }
    bool poseChanged = false;
    if (body) {
        int managedIndex = bodyManaged(name);
        if (managedIndex > -1) {
            poseChanged = true;
            sceneCollisionManager_->unregisterObject(body->getOpptCollisionObject()->getCollisionObject());
            body->setWorldPose(pose);
            body->updateCollisionObject();
            auto trans = body->getOpptCollisionObject()->getCollisionObject()->getTranslation();
            sceneCollisionManager_->registerObject(body->getOpptCollisionObject()->getCollisionObject());
            sceneCollisionManager_->update();
            collisionObjectMap_[body->getOpptCollisionObject()] = body->getName();
        } else {
            body->setWorldPose(pose);
        }

        if (executeCallback) {
            if (changePoseCallback_)
                changePoseCallback_(name, pose);
        }
    } else {
        WARNING("Trying to change pose of body '" + name + "' in scene, but body not present. Skipping...");
    }

    return poseChanged;
}

VectorBodyPtr SceneImpl::getBodies() const
{
    VectorBodyPtr bodies(bodies_.size(), nullptr);
    for (size_t i = 0; i != bodies.size(); ++i) {
        bodies[i] = bodies_[i].get();
    }
    return bodies;
}

Body* SceneImpl::getBody(std::string name) const
{
    for (size_t i = 0; i != bodies_.size(); i++) {
        auto n = bodies_[i]->getName();
        if (n == name) {
            return bodies_[i].get();
        } else if (n.find("_nestLink_") != std::string::npos && n.find(name) != std::string::npos) {
            return bodies_[i].get();
        }
    }

    return nullptr;
}

int SceneImpl::bodyManaged(const std::string& bodyName) const
{
    for (size_t i = 0; i < bodiesInManager_.size(); i++) {
        auto n = bodiesInManager_[i];
        if (n == bodyName) {
            return i;
        } else if (n.find("_nestLink_") != std::string::npos && n.find(bodyName) != std::string::npos) {
            return i;
        }
    }

    return -1;
}

void SceneImpl::makeDiscreteCollisionReport(oppt::CollisionReportSharedPtr& collisionReport)
{
    oppt::DiscreteCollisionReport* discreteCollisionReport =
        static_cast<oppt::DiscreteCollisionReport*>(collisionReport.get());
    fcl::CollisionRequest collisionRequest;
    collisionRequest.enable_contact = discreteCollisionReport->enableContact;
    collisionRequest.num_max_contacts = 10;
    fcl::CollisionResult collisionResult;
    CollisionData collisionData;
    collisionData.request = collisionRequest;
    collisionData.result = collisionResult;
    robotCollisionManager_->update();
    sceneCollisionManager_->collide(robotCollisionManager_.get(), &collisionData, defaultCollisionFunction);
    discreteCollisionReport->collides = collisionData.result.isCollision();
    /**for (size_t i = 0; i != collisionData.o1.size(); ++i) {
        discreteCollisionReport->collisionPairs.push_back(std::make_pair(collisionObjectMap_.at(collisionData.o1[i]),
                collisionObjectMap_.at(collisionData.o2[i])));
    }*/
}

void SceneImpl::makeContinuousCollisionReport(oppt::CollisionReportSharedPtr& collisionReport)
{
    oppt::ContinuousCollisionReport* continuousCollisionReport =
        static_cast<oppt::ContinuousCollisionReport*>(collisionReport.get());
    fcl::ContinuousCollisionRequest request(continuousCollisionReport->numIterations,
                                            0.0001,
                                            fcl::CCDM_LINEAR,
                                            fcl::GST_LIBCCD,
                                            fcl::CCDC_NAIVE);
    std::vector<fcl::CollisionObject*> robotCollisionObjects;
    robotCollisionManager_->getObjects(robotCollisionObjects);
    for (size_t i = 0; i < bodies_.size(); i++) {
        if (!bodies_[i]->isEnabled()) {
            continue;
        }
        FCLCollisionObject* bodyCollisionObject = bodies_[i]->getOpptCollisionObject()->getCollisionObject();
        for (size_t j = 0; j < continuousCollisionReport->tfGoal.size(); j++) {
            fcl::ContinuousCollisionResult result;
            fcl::continuousCollide(robotCollisionObjects_[j]->getCollisionObject(),
                                   continuousCollisionReport->tfGoal[j],
                                   bodyCollisionObject,
                                   bodyCollisionObject->getTransform(),
                                   request,
                                   result);
            if (result.is_collide) {
                continuousCollisionReport->collides = true;
                continuousCollisionReport->collidingBodies[0] = bodies_[i]->getName();
                continuousCollisionReport->collidingBodyIndex = j;
                continuousCollisionReport->collidingBody = robotCollisionObjects_[j]->getName();
                return;
            }
        }
    }
}

void SceneImpl::makeExtendedCollisionReport(oppt::ExtendedCollisionReportSharedPtr& collisionReport) const
{
    fcl::ContinuousCollisionRequest request(20,
                                            0.0001,
                                            fcl::CCDM_LINEAR,
                                            fcl::GST_LIBCCD,
                                            fcl::CCDC_NAIVE);
    for (size_t i = 0; i < bodies_.size(); i++) {
        if (!bodies_[i]->isEnabled()) {
            continue;
        }
        FCLCollisionObject * bodyCollisionObject = bodies_[i]->getOpptCollisionObject()->getCollisionObject();
        for (size_t j = 0; j < collisionReport->robotCollisionObjectsStart.size(); j++) {
            fcl::ContinuousCollisionResult result;
            fcl::continuousCollide(collisionReport->robotCollisionObjectsStart[j]->getCollisionObject(),
                                   collisionReport->robotCollisionObjectsGoal[j]->getCollisionObject()->getTransform(),
                                   bodyCollisionObject,
                                   bodyCollisionObject->getTransform(),
                                   request,
                                   result);
            if (result.is_collide) {
                collisionReport->timeOfContact = result.time_of_contact;
                collisionReport->collides = true;
                collisionReport->collidingBodies[0] = bodies_[i]->getName();
                collisionReport->collidingBodyIndex = j;
                collisionReport->collidingBody = collisionReport->robotCollisionObjectsGoal[j]->getName();
                return;
            }

        }
    }
}

void SceneImpl::setRemoveBodyCallback(std::function<void(std::string)> callbackFunction)
{
    removeBodyCallback_ = callbackFunction;
}

void SceneImpl::setAddBodyCallback(std::function<void(oppt::Body *const)> callbackFunction)
{
    addBodyCallback_ = callbackFunction;
}

void SceneImpl::setChangeBodyPoseCallback(std::function<void(const std::string&, const geometric::Pose&)> callbackFunction)
{
    changePoseCallback_ = callbackFunction;
}

void SceneImpl::setRobotCollisionObjects(VectorCollisionObjectPtr& robotCollisionObjects)
{
    robotCollisionObjects_ = robotCollisionObjects;
    robotCollisionManager_->clear();
    for (auto & robotCollisionObject : robotCollisionObjects_) {
        robotCollisionManager_->registerObject(robotCollisionObject->getCollisionObject());
        collisionObjectMap_[robotCollisionObject] = robotCollisionObject->getName();
    }
}

void SceneImpl::setCollisionInvariantRobotCollisionObjects(VectorCollisionObjectPtr &robotCollisionObjects) {
    collisionInvariantRobotCollisionObjects_ = robotCollisionObjects;
}

VectorCollisionObjectPtr SceneImpl::getCollisionInvariantRobotCollisionObjects() const {
    return collisionInvariantRobotCollisionObjects_;
}

VectorCollisionObjectPtr SceneImpl::getRobotCollisionObjects() const
{
    return robotCollisionObjects_;
}

std::string SceneImpl::toSDFString() const
{
    std::string sdfString = "";
    for (auto & body : bodies_) {
        sdfString += body->toSDFString();
    }

    return sdfString;
}

}
