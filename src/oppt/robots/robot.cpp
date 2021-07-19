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
#include "oppt/robotHeaders/robot.hpp"
#include "oppt/opptCore/ObservationReport.hpp"
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

#include "oppt/robotHeaders/RobotSpaceInformationFactory.hpp"
#include "oppt/robotHeaders/RobotSerializer.hpp"
#include "oppt/robotHeaders/RobotSDFParser.hpp"

#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace oppt
{

Robot::Robot(const std::string &worldFile,
             const std::string &robotName,
             const std::string &configFile,
             const std::string &prefix,
             unsigned int& threadId):
    robotName_(robotName),
    worldFile_(worldFile),
    configFile_(configFile),
    collisionsAllowed_(false),
    prefix_(prefix) {
    ikSolver_ = std::make_unique<DefaultIKSolver>(randomEngine_.get(), "", "", "");
    std::unique_ptr <options::OptionParser> robotConfigParser =
        RobotConfigOptions::makeParser(configFile);
    robotConfigParser->setOptions(&robotConfigOptions_);
    robotConfigParser->parseCfgFile(configFile);
    robotConfigParser->finalize();
    serializer_.reset(new RobotSerializer(worldFile));

    // Default implementation of a discrete collision function
    discreteCollisionFunction_ = [this](const RobotStateSharedPtr & state) {
        RobotStateSharedPtr denormalizedState = stateSpace_->denormalizeState(state);
        CollisionRequest collisionRequest;
        collisionRequest.enableContact = true;
        createRobotCollisionObjects_(denormalizedState);
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
        createRobotCollisionObjects_(denormalizedState2);
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

        createRobotCollisionObjects_(denormalizedState1);
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

bool Robot::init() {
    RobotSDFParser sdfParser;
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

    VectorGeometryUniquePtr visualGeometries = sdfParser.parseVisualGeometries(robotName_, worldFile_);
    for (auto& geometry : visualGeometries) {
        VectorString nameElems;
        oppt::split(geometry->getName(), "::", nameElems);
        if (nameElems[0] == robotName_)
            visualRobotGeometries_.push_back(std::move(geometry));
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
        static_cast<RobotSerializer*>(serializer_.get())->setWorld(gazeboInterface_->getWorld());
    }

    return true;
}

bool Robot::initializeViewer_(const std::string &robotName, const std::string &environmentFile, const unsigned int &frameRate)
{
#ifdef USE_RVIZ
    viewer_ = std::make_shared<ViewerPublisher>();
    static_cast<ViewerPublisher*>(viewer_.get())->setupViewer(robotName, environmentFile, frameRate);
#endif
    return true;
}

std::string Robot::getName() const
{
    return robotName_;
}

bool Robot::getCollisionsAllowed() const
{
    return collisionsAllowed_;
}

void Robot::setCollisionsAllowed(const bool& collisionsAllowed) const
{
    collisionsAllowed_ = collisionsAllowed;
    if (!collisionsAllowed and gazeboInterface_)
        gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);
}

void Robot::setRandomEngine(RandomEnginePtr randomEngine)
{
    randomEngine_ = randomEngine;
}

RandomEnginePtr Robot::getRandomEngine() const
{
    return randomEngine_;
}

VectorString Robot::getCollisionCheckedLinks() const {
    return collisionLinks_;
}

void Robot::createTransitionPlugin(const std::string& pluginFile,
                                   RobotEnvironment* robotEnvironment)
{
    transitionPlugin_ =
        TransitionPlugin::Create(pluginFile, "transitionPlugin");
    transitionPlugin_->robotEnvironment_ = robotEnvironment;
    transitionPluginFile_ = pluginFile;
}

void Robot::createObservationPlugin(const std::string& pluginFile,
                                    RobotEnvironment* robotEnvironment)
{
    observationPlugin_ =
        ObservationPlugin::Create(pluginFile, "observationPlugin");
    observationPlugin_->robotEnvironment_ = robotEnvironment;
    observationPluginFile_ = pluginFile;
}

void Robot::loadTransitionPlugin(const std::string& optionsFile)
{
    if (transitionPlugin_ == nullptr)
        ERROR("Transition plugin hasn't been created yet");
    if (!transitionPlugin_->load(optionsFile)) {
        ERROR("Transition plugin '" +
              transitionPluginFile_ +
              "' couldn't be loaded");
    }

    transitionPluginOptionsFile_ = optionsFile;
}

void Robot::loadObservationPlugin(const std::string& optionsFile)
{
    if (observationPlugin_ == nullptr)
        ERROR("Observation plugin hasn't been created yet");
    if (!observationPlugin_->load(optionsFile)) {
        ERROR("Observation plugin '" +
              observationPluginFile_ +
              "' couldn't be loaded");
    }

    observationPluginOptionsFile_ = optionsFile;
}

PropagationResultSharedPtr Robot::propagateState(PropagationRequestSharedPtr& propagationRequest)
{
    boost::this_thread::interruption_point();
    PropagationResultSharedPtr propagationResult = transitionPlugin_->propagateState(propagationRequest.get());
    propagationResult->previousState = propagationRequest->currentState.get();
    propagationResult->action = propagationRequest->action;
    //propagationResult->errorVector = propagationRequest->errorVector;
    return propagationResult;
}

ObservationResultSharedPtr Robot::makeObservationReport(ObservationRequestSharedPtr& observationRequest) const
{
    boost::this_thread::interruption_point();
    ObservationResultSharedPtr observationResult = observationPlugin_->getObservation(observationRequest.get());
    observationResult->state = observationRequest->currentState.get();
    observationResult->action = observationRequest->action;
    return observationResult;

}

oppt::CollisionReportSharedPtr Robot::makeDiscreteCollisionReport(const oppt::RobotStateSharedPtr& state) const
{
    if (!discreteCollisionFunction_)
        ERROR("No oppt::DiscreteCollisionFunction defined");
    return discreteCollisionFunction_(state);
}

CollisionReportSharedPtr Robot::makeDiscreteCollisionReportDirty() const
{
    if (!dirtyCollisionFunction_)
        ERROR("No oppt::DirtyCollisionFunction defined");
    return dirtyCollisionFunction_(nullptr);
}

oppt::CollisionReportSharedPtr Robot::makeContinuousCollisionReport(const oppt::RobotStateSharedPtr& state1,
        const oppt::RobotStateSharedPtr& state2,
        const unsigned int& numIterations) const
{
    if (!continuousCollisionFunction_)
        ERROR("No oppt::ContinuousCollisionFunction defined");
    return continuousCollisionFunction_(state1, state2, numIterations);
}

void Robot::setSerializer(oppt::SerializerSharedPtr &serializer) {
    serializer_ = serializer;
}


Serializer* Robot::getSerializer() const
{
    if (!serializer_)
        ERROR("Serializer is null");
    return serializer_.get();
}

oppt::StateSpaceSharedPtr Robot::getStateSpace() const
{
    return stateSpace_;
}

oppt::ObservationSpaceSharedPtr Robot::getObservationSpace() const
{
    return observationSpace_;
}

oppt::ActionSpaceSharedPtr Robot::getActionSpace() const
{
    if (!actionSpace_) {
        assert(false && "ACTION SPACE IS NULL");
    }
    return actionSpace_;
}

void Robot::setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo)
{
    if (!environmentInfo) {
        oppt::ERROR("Robot: setSenvironmentInfo: environmentInfo is null!!!");
    }

    environmentInfo_ = environmentInfo;

    VectorCollisionObjectPtr robotCollisionObjects(robotCollisionObjects_.size(), nullptr);
    VectorCollisionObjectPtr invariantRobotCollisionObjects(invariantRobotCollisionObjects_.size(), nullptr);
    for (size_t i = 0; i != robotCollisionObjects.size(); ++i) {
        robotCollisionObjects[i] = robotCollisionObjects_[i].get();
    }
    for (size_t i = 0; i != invariantRobotCollisionObjects.size(); ++i) {
        invariantRobotCollisionObjects[i] = invariantRobotCollisionObjects_[i].get();
    }

    environmentInfo_->getScene()->setRobotCollisionObjects(robotCollisionObjects);
    if (gazeboInterface_)
        gazeboInterface_->setInteractive(environmentInfo->isInteractive());
}

void Robot::addBodyCallback(Body *const body) {
    std::string sdfString = body->toSDFString();
    gazeboInterface_->addBodyFromSDFString(sdfString);
}

void Robot::removeBodyCallback(std::string name) {
    gazeboInterface_->removeBody(name);
}

void Robot::changeBodyPoseCallback(const std::string& name, const geometric::Pose& poseVec) {
    gazeboInterface_->changeModelPose(name, poseVec);
}

void Robot::initEnvironmentCallbacks() {
    if (environmentInfo_->isInteractive()) {
        std::function<void(oppt::Body *const)> addBodyFn =
            std::bind(&Robot::addBodyCallback, this, std::placeholders::_1);
        std::function<void(std::string)> removeBodyFn =
            std::bind(&Robot::removeBodyCallback, this, std::placeholders::_1);
        std::function<void(const std::string&, const geometric::Pose&)> changePoseFn =
            std::bind(&Robot::changeBodyPoseCallback, this, std::placeholders::_1, std::placeholders::_2);
        environmentInfo_->getScene()->setAddBodyCallback(addBodyFn);
        environmentInfo_->getScene()->setRemoveBodyCallback(removeBodyFn);
        environmentInfo_->getScene()->setChangeBodyPoseCallback(changePoseFn);
    }
}

void Robot::registerOnAddBodyCallback(std::function<void(const std::string&)>& callback) {
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnAddBodyCallback(callback);
}

void Robot::registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback) {
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnRemoveBodyCallback(callback);
}

void Robot::registerOnPoseChangedCallback(std::function<void(const std::string&, const geometric::Pose&)>& callback) {
    if (environmentInfo_->isInteractive() and gazeboInterface_)
        gazeboInterface_->registerOnPoseChangedCallback(callback);
}

void Robot::setupViewer(const std::string &robotName, const std::string &environmentFile, const unsigned int &frameRate)
{
#ifdef USE_RVIZ
    if (viewer_) {
        static_cast<ViewerPublisher*>(viewer_.get())->setupViewer(robotName, environmentFile, frameRate);
        static_cast<ViewerPublisher*>(viewer_.get())->updateFromEnvironmentInfo(environmentInfo_);
    }
#endif
}

const std::string Robot::getRobotName() const
{
    return robotName_;
}

FloatType Robot::calcLikelihood(const RobotStateSharedPtr &state,
                                const Action *action,
                                const Observation *observation)
{
    return observationPlugin_->calcLikelihood(state, action, observation);
}

void Robot::setGazeboInterface(GazeboInterface* const gazeboInterface) {
    gazeboInterface_ = gazeboInterface;
}

bool Robot::normalizedSpaces() const {
    return normalizedSpaces_;
}

TransitionPlugin* const Robot::getTransitionPlugin() const {
    return transitionPlugin_.get();
}

ObservationPlugin* const Robot::getObservationPlugin() const {
    return observationPlugin_.get();
}

void Robot::setDiscreteCollisionFunction(DiscreteCollisionFunction discreteCollisionFunction) {
    discreteCollisionFunction_ = discreteCollisionFunction;
}

void Robot::setContinuousCollisionFunction(ContinuousCollisionFunction continuousCollisionFunction) {
    continuousCollisionFunction_ = continuousCollisionFunction;
}

void Robot::setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction) {
    dirtyCollisionFunction_ = dirtyCollisionFunction;
    if (gazeboInterface_)
        gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);
}

void Robot::createRobotCollisionObjects_(const oppt::RobotStateSharedPtr& state) const
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

ViewerBase* const Robot::getViewer() const
{
    if (viewer_)
        return viewer_.get();
    return nullptr;
}

void Robot::setIKSolver(IKSolverUniquePtr ikSolver) {
    ikSolver_ = std::move(ikSolver);
}

IKSolver *const Robot::getIKSolver() const {
    if (ikSolver_)
        return ikSolver_.get();
    return nullptr;
}

std::pair<bool, VectorFloat> Robot::getIKSolution(const geometric::Pose &pose,
        const unsigned int &numAttempts,
        const VectorFloat &qInit) const {
    if (ikSolver_)
        return ikSolver_->solve(pose, numAttempts, qInit);
    return std::make_pair(false, VectorFloat());
}

bool Robot::makeStateSpace(StateSpaceInfo& stateSpaceInfo) {
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
    serializer_->as<RobotSerializer>()->setStateSpace(stateSpace_.get());
    return true;
}

bool Robot::makeActionSpace(ActionSpaceInfo& actionSpaceInfo) {
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

bool Robot::makeObservationSpace(ObservationSpaceInfo& observationSpaceInfo) {
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

void Robot::updateViewer(const oppt::RobotStateSharedPtr& state,
                         const std::vector<oppt::RobotStateSharedPtr>& particles,
                         const FloatType& particleOpacity,
                         const bool& keepParticles,
                         const bool& deleteExistingParticles) {
#ifdef USE_RVIZ
    if (viewer_ && viewer_->viewerRunning()) {
        VectorGeometryPtr geometries;
        VectorGeometryUniquePtr copiedGeometries;
        VectorString correspondingLinkNames;
        std::vector<geometric::Pose> relativePoses;
        std::unordered_map<std::string, geometric::Pose> linkStatePoses;

        // Lambda to obtain the link poses in a gazebo world state
        // linkStatePoses is a map from link names to link world poses
        auto getLinkStatePoses = [](const oppt::RobotStateSharedPtr & state,
        std::unordered_map<std::string, geometric::Pose> &linkStatePoses) {
            if (!(state->getGazeboWorldState()))
                return;
            auto modelStates = state->getGazeboWorldState()->getWorldState()->GetModelStates();
            for (auto &modelState : modelStates) {
                auto linkStates = modelState.second.GetLinkStates();
                for (auto &linkState : linkStates) {
#ifdef GZ_GT_7
                    linkStatePoses[modelState.first + "::" + linkState.first] = geometric::Pose(linkState.second.Pose());
#else
                    linkStatePoses[modelState.first + "::" + linkState.first] = geometric::Pose(linkState.second.GetPose());
#endif
                }
            }
        };

        getLinkStatePoses(state, linkStatePoses);
        auto bodies = environmentInfo_->getScene()->getBodies();
        copiedGeometries.reserve(bodies.size() + visualRobotGeometries_.size());
        for (auto &body : bodies) {
            bool bodyInLinkStates = false;
            if (linkStatePoses.find(body->getScopedName()) != linkStatePoses.end()) {
                bodyInLinkStates = true;
            }

            auto visualGeometries = body->getVisualGeometries();
            for (auto &visualGeometry : visualGeometries) {
                auto copiedGeometry = visualGeometry->shallowCopy();
                geometric::Pose relativePose;

                // Check if the environment body is represented in the gazebo world state links.
                // If yes, set the pose of the visual geometry to the link state pose + the relative geometry pose
                if (bodyInLinkStates) {
                    // The relative pose of the visual geometry to the body world pose
                    relativePose = copiedGeometry->getWorldPose() - body->getWorldPose();
                    copiedGeometry->setWorldPose(relativePose + linkStatePoses.at(body->getScopedName()));
                }

                geometries.push_back(copiedGeometry.get());
                copiedGeometries.push_back(std::move(copiedGeometry));
                correspondingLinkNames.push_back(body->getScopedName());
                relativePoses.push_back(relativePose);
            }
        }

        for (auto &visualRobotGeometry : visualRobotGeometries_) {
            std::string scopedName = visualRobotGeometry->getScopedName();
            VectorString scopedNameElems;
            split(scopedName, "::", scopedNameElems);
            std::string correspondingLinkName = "";
            for (size_t i = 0; i != scopedNameElems.size() - 2; ++i) {
                correspondingLinkName += scopedNameElems[i] + "::";
            }

            correspondingLinkName += scopedNameElems[scopedNameElems.size() - 2];
            if (linkStatePoses.find(correspondingLinkName) != linkStatePoses.end()) {
                auto copiedGeometry = visualRobotGeometry->shallowCopy();

                // getWorldPose() of visualRobotGeometry is the relative pose to its parent link, not the
                // actual world pose
                auto relativePose = visualRobotGeometry->getWorldPose();
                copiedGeometry->setWorldPose(relativePose + linkStatePoses.at(correspondingLinkName));
                geometries.push_back(copiedGeometry.get());
                copiedGeometries.push_back(std::move(copiedGeometry));
                correspondingLinkNames.push_back(correspondingLinkName);
                relativePoses.push_back(relativePose);
            }
        }


        std::vector<std::vector<geometric::Pose>> particlePoses;
        particlePoses.reserve(particles.size());
        for (size_t i = 0; i != particles.size(); ++i) {
            std::vector<geometric::Pose> particlePose;
            particlePose.reserve(copiedGeometries.size());
            getLinkStatePoses(particles[i], linkStatePoses);
            for (size_t i = 0; i != copiedGeometries.size(); ++i) {
                geometric::Pose pPose = copiedGeometries[i]->getWorldPose();
                if (linkStatePoses.find(correspondingLinkNames[i]) != linkStatePoses.end()) {
                    pPose = relativePoses[i] + linkStatePoses.at(correspondingLinkNames[i]);
                }

                particlePose.push_back(pPose);
            }

            particlePoses.push_back(particlePose);
        }

        static_cast<ViewerPublisher*>(viewer_.get())->drawGeometries(geometries,
                particlePoses,
                particleOpacity,
                keepParticles,
                deleteExistingParticles);
    }
#endif
}

}
