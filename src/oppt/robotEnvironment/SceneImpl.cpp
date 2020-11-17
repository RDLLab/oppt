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
#include "oppt/opptCore/CollisionRequest.hpp"
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
        bodiesInManager_[bodyPtr->getName()] = body.get();
        //bodiesInManager_.push_back(bodyPtr->getName());
        auto opptCollisionObjects = bodyPtr->getOpptCollisionObjects();
        for (auto &opptCollisionObject : opptCollisionObjects) {
            sceneCollisionManager_->registerObject(opptCollisionObject->getFCLCollisionObject());
            collisionObjectMap_[opptCollisionObject] = bodyPtr->getName();
        }
        //sceneCollisionManager_->registerObject(bodyPtr->getOpptCollisionObject()->getCollisionObject());
        sceneCollisionManager_->update();
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
            auto opptCollisionObjects = bodies_[i]->getOpptCollisionObjects();
            if (bodyManaged(body->getName())) {
                for (auto &collisionObject : opptCollisionObjects) {
                    sceneCollisionManager_->unregisterObject(collisionObject->getFCLCollisionObject());
                }

                sceneCollisionManager_->update();
                bodiesInManager_.erase(body->getName());
            }

            for (auto &collisionObject : opptCollisionObjects) {
                if (collisionObjectMap_.find(collisionObject) != collisionObjectMap_.end())
                    collisionObjectMap_.erase(collisionObject);
            }

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
        body->setWorldPose(pose);
        body->updateCollisionObjects();
        if (bodyManaged(name)) {
            poseChanged = true;
            //body->setWorldPose(pose);
            //body->updateCollisionObjects();
            auto opptCollisionObjects = body->getOpptCollisionObjects();
            for (auto &collisionObject : opptCollisionObjects) {
                sceneCollisionManager_->update(collisionObject->getFCLCollisionObject());
            }
            //sceneCollisionManager_->update(body->getOpptCollisionObject()->getCollisionObject());
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

void SceneImpl::changeBodyPoses(const VectorString& names, const std::vector<geometric::Pose>& poses, bool executeCallback) {
    std::vector<FCLCollisionObject *> fclCollisionObjectsToUpdate;
    for (size_t i = 0; i != names.size(); ++i) {
        Body *body = getBody(names[i]);
        if (body) {
            body->setWorldPose(poses[i]);
            body->updateCollisionObjects();
            auto opptCollisionObjects = body->getOpptCollisionObjects();
            for (auto &collisionObject : opptCollisionObjects) {
                fclCollisionObjectsToUpdate.push_back(collisionObject->getFCLCollisionObject());
            }
        }
    }

    sceneCollisionManager_->update(fclCollisionObjectsToUpdate);
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

bool SceneImpl::bodyManaged(const std::string& bodyName) const
{
    if (bodiesInManager_.find(bodyName) != bodiesInManager_.end())
        return true;
    return false;
}

CollisionReportSharedPtr SceneImpl::makeDiscreteCollisionReport(const CollisionRequest *collisionRequest)
{
    CollisionReportSharedPtr collisionReport(new CollisionReport);
    fcl::CollisionRequest fclCollisionRequest;
    fclCollisionRequest.enable_contact = collisionRequest->enableContact;
    fclCollisionRequest.num_max_contacts = 10;
    fcl::CollisionResult collisionResult;
    CollisionData collisionData;
    collisionData.request = fclCollisionRequest;
    collisionData.result = collisionResult;
    robotCollisionManager_->update();
    sceneCollisionManager_->collide(robotCollisionManager_.get(), &collisionData, defaultCollisionFunction);
    collisionReport->collides = collisionData.result.isCollision();
    for (size_t i = 0; i != collisionData.o1.size(); ++i) {
        auto fclUserData = collisionData.o1[i]->getUserData();
        //auto collObject1 = ((OpptCollisionObjectData *)(collisionData.o2[i]->getUserData()))->opptCollisionObject;
        collisionReport->collisionPairs.push_back(
            std::pair<std::string, std::string>(((OpptCollisionObjectData *)(collisionData.o2[i]->getUserData()))->opptCollisionObject->getCollisionGeometry()->getName(),
                                                ((OpptCollisionObjectData *)(collisionData.o1[i]->getUserData()))->opptCollisionObject->getCollisionGeometry()->getName()));
        // TODO: add contacts if requested

    }

    return collisionReport;
}

CollisionReportSharedPtr SceneImpl::makeContinuousCollisionReport(const CollisionRequest *collisionRequest)
{
    CollisionReportSharedPtr collisionReport(new CollisionReport);
    fcl::ContinuousCollisionRequest request(collisionRequest->numIterations,
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

        auto opptCollisionObjects = bodies_[i]->getOpptCollisionObjects();
        bool timeOfContactSet = false;
        for (size_t j = 0; j != collisionRequest->tfGoal.size(); j++) {
            for (auto &opptCollisionObject : opptCollisionObjects) {
                fcl::ContinuousCollisionResult result;
                fcl::continuousCollide(robotCollisionObjects_[j]->getFCLCollisionObject(),
                                       collisionRequest->tfGoal[j],
                                       opptCollisionObject->getFCLCollisionObject(),
                                       opptCollisionObject->getFCLCollisionObject()->getTransform(),
                                       request,
                                       result);
                if (result.is_collide) {
                    collisionReport->collides = true;
                    if (!timeOfContactSet) {
                        collisionReport->timeOfContact = result.time_of_contact;
                        timeOfContactSet = true;
                    }
                    //collisionReport->collidingBodies[0] = bodies_[i]->getName();
                    collisionReport->collisionPairs.push_back(
                        std::pair<std::string, std::string>({robotCollisionObjects_[j]->getCollisionGeometry()->getName(),
                                                            opptCollisionObject->getCollisionGeometry()->getName()
                                                            }));
                }
            }
        }
    }

    return collisionReport;
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
        robotCollisionManager_->registerObject(robotCollisionObject->getFCLCollisionObject());
        collisionObjectMap_[robotCollisionObject] = robotCollisionObject->getCollisionGeometry()->getName();
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
