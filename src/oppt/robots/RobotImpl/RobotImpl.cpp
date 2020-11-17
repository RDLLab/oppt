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
#include "oppt/robotHeaders/RobotImpl/RobotImpl.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImplSDFParser.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImplSpaceInformationFactory.hpp"
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace oppt
{

RobotImpl::RobotImpl(const std::string &worldFile,
                     const std::string &robotName,
                     const std::string &configFile,
                     const std::string &prefix,
                     unsigned int& threadId):
    Robot(worldFile, robotName, configFile, prefix, threadId),
    worldFile_(worldFile),
    collisionLinks_(),
    linkVisualGeometryMap_(),
    dof_(0),
    robotConfigOptions_()
{
    std::unique_ptr <options::OptionParser> robotConfigParser =
        RobotConfigOptions::makeParser(configFile);
    robotConfigParser->setOptions(&robotConfigOptions_);
    robotConfigParser->parseCfgFile(configFile);
    robotConfigParser->finalize();
    serializer_.reset(new RobotImplSerializer(worldFile));

    // Default implementation of a discrete collision function
    discreteCollisionFunction_ = [this](const RobotStateSharedPtr & state) {
        RobotStateSharedPtr denormalizedState = stateSpace_->denormalizeState(state);
        CollisionRequest collisionRequest;
        collisionRequest.enableContact = true;
        createRobotCollisionObjects(denormalizedState);
        return environmentInfo_->getScene()->makeDiscreteCollisionReport(&collisionRequest);
    };

    // Default implementation of a continuous collision function
    continuousCollisionFunction_ = [this](const RobotStateSharedPtr & state1,
                                          const RobotStateSharedPtr & state2,
    const unsigned int& numIterations) {
        RobotStateSharedPtr denormalizedState1 = stateSpace_->denormalizeState(state1);
        RobotStateSharedPtr denormalizedState2 = stateSpace_->denormalizeState(state2);
        CollisionRequest collisionRequest;
        collisionRequest.tfGoal = VectorFCLTransform3f(collisionLinks_.size());
        collisionRequest.numIterations = numIterations;

        // Get the goal transforms
        createRobotCollisionObjects(denormalizedState2);
        unsigned int idx = 0;
        for (auto& robotCollisionObject : robotCollisionObjects_) {
            // Unfortunately, we need to copy the transformation object here
            FCLTransform3f tf = robotCollisionObject->getFCLCollisionObject()->getTransform();
            fcl::Vec3f vec = tf.getTranslation();
            fcl::Vec3f vec2(vec[0], vec[1], vec[2]);
            fcl::Quaternion3f quat = tf.getQuatRotation();
            fcl::Quaternion3f quat2(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
            FCLTransform3f tf2(quat2, vec2);
            collisionRequest.tfGoal[idx] = tf2;
            idx++;
        }

        createRobotCollisionObjects(denormalizedState1);
        return environmentInfo_->getScene()->makeContinuousCollisionReport(&collisionRequest);
    };

    // Default implementation of a dirty collision function
    dirtyCollisionFunction_ = [this](void *const data) {
        auto linkPoses = gazeboInterface_->getLinksCollisionWorldPosesDirty(collisionLinks_);
        for (size_t i = 0; i != linkPoses.size(); ++i) {
            fcl::Quaternion3f fclQuat(linkPoses[i].orientation.w(),
                                      linkPoses[i].orientation.x(),
                                      linkPoses[i].orientation.y(),
                                      linkPoses[i].orientation.z());
            fcl::Vec3f translationVector(linkPoses[i].position.x(),
                                         linkPoses[i].position.y(),
                                         linkPoses[i].position.z());
            fcl::Transform3f trans(fclQuat, translationVector);
            robotCollisionObjects_[i]->getFCLCollisionObject()->setTransform(trans);
            if (robotCollisionObjects_[i]->getFCLCollisionObject()->collisionGeometry())
                robotCollisionObjects_[i]->getFCLCollisionObject()->computeAABB();

        }

        CollisionRequest collisionRequest;
        collisionRequest.enableContact = false;
        return environmentInfo_->getScene()->makeDiscreteCollisionReport(&collisionRequest);
    };
}

bool RobotImpl::init()
{
    RobotImplSDFParser sdfParser;
    VectorGeometryUniquePtr robotLinkCollisionGeometries = std::move(sdfParser.parseCollisionGeometries(robotName_, worldFile_));
    VectorGeometryUniquePtr invariantRobotCollisionGeometries;
    VectorString collisionInvariantLinks = robotConfigOptions_.collisionInvariantLinks;
    std::unordered_map<std::string, std::vector<GeometryUniquePtr>> linkCollisionGeometryMap;
    for (auto& geometry : robotLinkCollisionGeometries) {
        bool contains = false;
        for (auto& collisionInvariantLink : collisionInvariantLinks) {
            if (geometry->getName().find(collisionInvariantLink) != std::string::npos) {
                contains = true;
            }
        }

        VectorString elems;
        split(geometry->getName(), "::", elems);
        if (elems.size() < 2)
            ERROR("Link has no parent model");
        std::string linkName = elems[0] + "::" + elems[1];
        if (!contains) {
            if (linkCollisionGeometryMap.find(linkName) == linkCollisionGeometryMap.end())
                linkCollisionGeometryMap[linkName] = std::vector<GeometryUniquePtr>();
            linkCollisionGeometryMap.at(linkName).push_back(std::move(geometry));
        } else {
            invariantRobotCollisionGeometries.push_back(std::move(geometry));
        }
    }

    VectorGeometryUniquePtr robotLinkVisualGeometries = sdfParser.parseVisualGeometries(robotName_, worldFile_);
    for (auto& geometry : robotLinkVisualGeometries) {
        linkVisualGeometryMap_[geometry->getName()] = std::move(geometry);
    }

    for (auto &collisionEntry : linkCollisionGeometryMap) {
        for (auto &collisionGeometry : collisionEntry.second) {
            OpptCollisionObjectUniquePtr opptCollisionObject(new OpptCollisionObject(std::move(collisionGeometry)));
            robotCollisionObjects_.push_back(std::move(opptCollisionObject));
        }
    }

    // Make the collision objects for the collision invariant links
    for (auto &linkGeometry : invariantRobotCollisionGeometries) {
        OpptCollisionObjectUniquePtr opptCollisionObject(new OpptCollisionObject(std::move(linkGeometry)));
        invariantRobotCollisionObjects_.push_back(std::move(opptCollisionObject));
    }

    if (gazeboInterface_) {
        VectorString robotLinksInWorld = gazeboInterface_->getRobotLinkNames();
        for (auto& link : collisionInvariantLinks) {
            if (!contains(robotLinksInWorld, link))
                ERROR("link '" + link + "' not defined in your robot model");
        }

        //TODO: Filter out dimensionless links
        collisionLinks_ = VectorString();
        for (auto &entry : linkCollisionGeometryMap) {
            collisionLinks_.push_back(entry.first);
        }

        gazeboInterface_->setCollisionCheckedLinks(collisionLinks_);
        static_cast<RobotImplSerializer*>(serializer_.get())->setWorld(gazeboInterface_->getWorld());
    }
    
    return true;
}

VectorString RobotImpl::getCollisionCheckedLinks() const {
    return collisionLinks_;
}

bool RobotImpl::makeStateSpace(StateSpaceInfo& stateSpaceInfo)
{
    const StateSpaceInformationPtr stateSpaceInformation =
        SpaceInformationFactory::makeStateSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    VectorFloat lowerLimits, upperLimits;
    if (gazeboInterface_) {
        gazeboInterface_->setStateSpaceInformation(stateSpaceInformation);
        gazeboInterface_->getStateLimits(lowerLimits, upperLimits);
    }

    stateSpaceInfo.spaceVariables = stateSpaceInformation->orderedVariables;

    // Limits for additional dimensions
    for (auto& lowerUpper : stateSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper[0]);
        upperLimits.push_back(lowerUpper[1]);
    }

    stateSpaceInfo.numDimensions = lowerLimits.size();
    stateSpace_ = std::make_shared<VectorStateSpace>(stateSpaceInfo);
    if (stateSpace_->getNumDimensions() == 0)
        WARNING("The state space has zero dimensions. "
                "Have you forgotten to specify states in your robot configuration file?");
    DistanceMetric distanceMetric = oppt::EuclideanMetric();
    stateSpace_->setDistanceMetric(distanceMetric);
    stateSpace_->as<VectorStateSpace>()->makeStateLimits(lowerLimits, upperLimits);
    dof_ = stateSpaceInformation->jointPositions.size();
    dof_ += stateSpaceInformation->containedLinkPositionsX.size();
    dof_ += stateSpaceInformation->containedLinkPositionsY.size();
    dof_ += stateSpaceInformation->containedLinkPositionsZ.size();
    dof_ += 6 * stateSpaceInformation->containedLinkPoses.size();
    serializer_->as<RobotImplSerializer>()->setStateSpace(stateSpace_.get());
    //static_cast<RobotImplIntegrator*>(integrator.get())->setStateSpace(stateSpace_);
    return true;
}

bool RobotImpl::makeActionSpace(ActionSpaceInfo& actionSpaceInfo)
{
    const ActionSpaceInformationPtr actionSpaceInformation =
        SpaceInformationFactory::makeActionSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    VectorFloat lowerLimits, upperLimits;
    if (gazeboInterface_) {
        gazeboInterface_->setActionSpaceInformation(actionSpaceInformation);
        gazeboInterface_->getActionLimits(lowerLimits, upperLimits);
    }

    actionSpaceInfo.spaceVariables = actionSpaceInformation->orderedVariables;

    // Limits for position increment controlled joints
    for (auto& lowerUpper : actionSpaceInformation->lowerUpperLimitsJointPositionIncrements) {
        lowerLimits.push_back(lowerUpper.first);
        upperLimits.push_back(lowerUpper.second);
    }

    // Limits for additional dimensions
    for (auto& lowerUpper : actionSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper.first);
        upperLimits.push_back(lowerUpper.second);
    }

    actionSpaceInfo.numDimensions = lowerLimits.size();
    actionSpace_ = std::make_shared<oppt::ContinuousVectorActionSpace>(actionSpaceInfo);
    actionSpace_->as<ContinuousVectorActionSpace>()->makeActionLimits(lowerLimits, upperLimits);

    if (actionSpace_->getNumDimensions() == 0)
        WARNING("The action space has zero dimensions. "
                "Have you forgotten to specify actions in your robot configuration file?");
    return true;
}

bool RobotImpl::makeObservationSpace(ObservationSpaceInfo& observationSpaceInfo)
{
    const ObservationSpaceInformationPtr observationSpaceInformation =
        SpaceInformationFactory::makeObservationSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    VectorFloat lowerLimits, upperLimits;
    if (gazeboInterface_) {
        gazeboInterface_->setObservationSpaceInformation(observationSpaceInformation);
        gazeboInterface_->getObservationLimits(lowerLimits, upperLimits, robotConfigOptions_.ignoreGazeboSensors);
    }
    observationSpaceInfo.spaceVariables = observationSpaceInformation->orderedVariables;

    // Limits for additional dimensions
    for (auto& lowerUpper : observationSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper[0]);
        upperLimits.push_back(lowerUpper[1]);
    }

    observationSpaceInfo.numDimensions = lowerLimits.size();
    if (gazeboInterface_)
        gazeboInterface_->setObservationSpaceDimension(observationSpaceInfo.numDimensions);
    observationSpace_ = std::make_shared<VectorObservationSpace>(observationSpaceInfo);
    if (observationSpace_->getNumDimensions() == 0)
        WARNING("The observation space has zero dimensions. "
                "Have you forgotten to specify observations in your robot configuration file?");
    observationSpace_->as<VectorObservationSpace>()->makeObservationLimits(lowerLimits, upperLimits);
    return true;
}

void RobotImpl::createRobotCollisionObjects(const oppt::RobotStateSharedPtr& state) const
{
    auto linkPoses = gazeboInterface_->getLinksCollisionWorldPoses(state, collisionLinks_);
    for (size_t i = 0; i != linkPoses.size(); ++i) {
        fcl::Quaternion3f fclQuat(linkPoses[i].orientation.w(),
                                  linkPoses[i].orientation.x(),
                                  linkPoses[i].orientation.y(),
                                  linkPoses[i].orientation.z());
        fcl::Vec3f translationVector(linkPoses[i].position.x(),
                                     linkPoses[i].position.y(),
                                     linkPoses[i].position.z());
        FCLTransform3f trans(fclQuat, translationVector);
        robotCollisionObjects_[i]->getFCLCollisionObject()->setTransform(trans);
        if (robotCollisionObjects_[i]->getFCLCollisionObject()->collisionGeometry())
            robotCollisionObjects_[i]->getFCLCollisionObject()->computeAABB();
    }
}

CollisionReportSharedPtr RobotImpl::makeDiscreteCollisionReportDirty() const
{
    if (!dirtyCollisionFunction_)
        ERROR("No oppt::DirtyCollisionFunction defined");
    return dirtyCollisionFunction_(nullptr);
}

unsigned int RobotImpl::getDOF() const
{
    return dof_;
}

void RobotImpl::updateViewer(const oppt::RobotStateSharedPtr& state,
                             const std::vector<oppt::RobotStateSharedPtr>& particles,
                             const FloatType& particleOpacity,
                             const bool& keepParticles,
                             const bool& deleteExistingParticles)
{
#ifdef USE_RVIZ
    if (viewer_ && viewer_->viewerRunning()) {
        //state->getGazeboWorldState();
        RobotStateSharedPtr denormalizedState = stateSpace_->denormalizeState(state);
        VectorFloat stateVec = denormalizedState->as<VectorState>()->asVector();
        VectorGeometryUniquePtr copiedRobotGeometries;
        VectorGeometryPtr robotGeometries;

        VectorString visualNames(linkVisualGeometryMap_.size());
        size_t k = 0;
        for (auto& linkGeomEntry : linkVisualGeometryMap_) {
            visualNames[k] = linkGeomEntry.first;
            k++;
        }

        std::vector<geometric::Pose> visualPoses;
        gazeboInterface_->getRobotVisualPoses(stateVec, visualNames, visualPoses, denormalizedState->getGazeboWorldState());
        for (size_t i = 0; i < visualNames.size(); i++) {
            const GeometrySharedPtr geomInMap = linkVisualGeometryMap_.at(visualNames[i]);
            GeometryUniquePtr geom = std::move(geomInMap->shallowCopy());
            geom->setWorldPose(visualPoses[i]);
            robotGeometries.push_back(geom.get());
            copiedRobotGeometries.push_back(std::move(geom));
        }

        std::vector<std::vector<geometric::Pose>> particlePoses(particles.size());
        for (size_t i = 0; i < particles.size(); i++) {
            std::vector<geometric::Pose> particlePose;
            denormalizedState = stateSpace_->denormalizeState(particles[i]);
            stateVec = denormalizedState->as<VectorState>()->asVector();
            gazeboInterface_->getRobotVisualPoses(stateVec, visualNames, visualPoses, denormalizedState->getGazeboWorldState());
            for (size_t j = 0; j != visualNames.size(); j++) {
                particlePose.push_back(visualPoses[j]);
            }

            particlePoses.push_back(particlePose);
        }

        static_cast<ViewerPublisher*>(viewer_.get())->drawGeometries(robotGeometries,
                particlePoses,
                particleOpacity,
                keepParticles,
                deleteExistingParticles);

        // Make sure that the world state is the same as before updating the viewer
        gazeboInterface_->setWorldState(state->getGazeboWorldState().get());
    }
#endif
}

void RobotImpl::setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo)
{
    Robot::setEnvironmentInfo(environmentInfo);
    if (gazeboInterface_)
        gazeboInterface_->setInteractive(environmentInfo->isInteractive());
}

void RobotImpl::initEnvironmentCallbacks()
{
    if (environmentInfo_->isInteractive()) {
        std::function<void(oppt::Body *const)> addBodyFn =
            std::bind(&RobotImpl::addBodyCallback, this, std::placeholders::_1);
        std::function<void(std::string)> removeBodyFn =
            std::bind(&RobotImpl::removeBodyCallback, this, std::placeholders::_1);
        std::function<void(const std::string&, const geometric::Pose&)> changePoseFn =
            std::bind(&RobotImpl::changeBodyPoseCallback, this, std::placeholders::_1, std::placeholders::_2);
        environmentInfo_->getScene()->setAddBodyCallback(addBodyFn);
        environmentInfo_->getScene()->setRemoveBodyCallback(removeBodyFn);
        environmentInfo_->getScene()->setChangeBodyPoseCallback(changePoseFn);
    }
}

void RobotImpl::registerOnAddBodyCallback(std::function<void(const std::string&)>& callback)
{
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnAddBodyCallback(callback);
}

void RobotImpl::registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback)
{
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnRemoveBodyCallback(callback);
}

void RobotImpl::registerOnPoseChangedCallback(std::function < void(const std::string&,
        const geometric::Pose&) >& callback)
{
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnPoseChangedCallback(callback);
}

void RobotImpl::addBodyCallback(Body *const body)
{
    std::string sdfString = body->toSDFString();
    gazeboInterface_->addBodyFromSDFString(sdfString);
}

void RobotImpl::removeBodyCallback(std::string name)
{
    gazeboInterface_->removeBody(name);
}

void RobotImpl::changeBodyPoseCallback(const std::string& name, const geometric::Pose& poseVec)
{
    gazeboInterface_->changeModelPose(name, poseVec);
}

void RobotImpl::setCollisionsAllowed(const bool& collisionsAllowed) const
{
    Robot::setCollisionsAllowed(collisionsAllowed);
    if (!collisionsAllowed and gazeboInterface_)
        gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);
}

void RobotImpl::setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction)
{
    dirtyCollisionFunction_ = dirtyCollisionFunction;
    if (gazeboInterface_)
        gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);

}

}
