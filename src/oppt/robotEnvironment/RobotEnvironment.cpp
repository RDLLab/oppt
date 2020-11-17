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
#include "include/RobotEnvironment.hpp"
#include <boost/lexical_cast.hpp>
#include "oppt/plugin/Plugin.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "include/SceneImpl.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImpl.hpp"
#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace oppt
{

RobotEnvironment::RobotEnvironment():
    robot_(nullptr),
    robotName_(""),
    configPath_(""),
    worldFile_(""),
    environmentInfo_(nullptr),
    scene_(nullptr),
    onAddBodyFn_(nullptr),
    onRemoveBodyFn_(nullptr),
    onEnvironmentChangedFns_(),
    threadId_(0),
    environmentChanges_(std::make_shared<EnvironmentChanges>()),
    registeredRobotEnvironments_()
{

}

RobotEnvironment::~RobotEnvironment()
{
    robot_.reset();
}

const std::string RobotEnvironment::getPrefix() const
{
    return prefix_;
}

void RobotEnvironment::setProblemEnvironmentChangeHandlerFn(ProblemEnvironmentChangeHandlerFn problemEnvironmentChangeHandlerFn)
{
    problemEnvironmentChangeHandlerFn_ = problemEnvironmentChangeHandlerFn;
}

void RobotEnvironment::initializeEnvironmentCallbacks()
{
    if (prefix_ == "exec") {
        onAddBodyFn_ =
            std::function<void(const std::string&)>(std::bind(&RobotEnvironment::onAddBody,
                    this,
                    std::placeholders::_1));
        robot_->registerOnAddBodyCallback(onAddBodyFn_);
        onRemoveBodyFn_ =
            std::function<void(const std::string&)>(std::bind(&RobotEnvironment::onRemoveBody,
                    this,
                    std::placeholders::_1));
        robot_->registerOnRemoveBodyCallback(onRemoveBodyFn_);
        onBodyPoseChangedFn_ =
            std::function<void(const std::string&, const geometric::Pose&)>(std::bind(&RobotEnvironment::onBodyPoseChanged,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        robot_->registerOnPoseChangedCallback(onBodyPoseChangedFn_);
    }

    auto updateSceneFn = [this]() {
        auto linkPoses = gazeboInterface_->getLinkPosesDirty();
        scene_->changeBodyPoses(linkPoses.first, linkPoses.second);
    };

    //auto updateSceneFn = scene_->getUpdateSceneFn();
    if (gazeboInterface_)
        gazeboInterface_->setUpdateSceneFn(updateSceneFn);
}

void RobotEnvironment::addEnvironmentChange(const EnvironmentChangeSharedPtr& environmentChange)
{
    environmentChanges_->addChange(environmentChange);
}

void RobotEnvironment::onAddBody(const std::string& str)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        EnvironmentChangeSharedPtr change = std::make_shared<BodyAddedChange>(str);
        //environmentChanges_->addChange(change);

        // Inform the solver about the changes

        // This needs to be changed such that ProblemEnvironment gets informed
        // about the change
        // which includes removing the addChange() line above
        // since this will be done by the ProblemEnvironment
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::BODY_ADDED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }

}

void RobotEnvironment::onRemoveBody(const std::string& str)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        std::string bodyName = str;

        // Deal with scoped names
        if (bodyName.find("::") != std::string::npos) {
            VectorString elems;
            split(str, "::", elems);
            bodyName = elems[elems.size() - 1];
        }
        EnvironmentChangeSharedPtr change = std::make_shared<BodyRemovedChange>(bodyName);
        //environmentChanges_->addChange(change);
        // Inform the solver about the changes
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::BODY_REMOVED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }
}

void RobotEnvironment::onBodyPoseChanged(const std::string& str, const geometric::Pose& pose)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        EnvironmentChangeSharedPtr change = std::make_shared<BodyPoseChange>(str, pose);
        //environmentChanges_->addChange(change);
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::BODY_POSE_CHANGED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }
}

void RobotEnvironment::applyChanges()
{
    // Applies the environment changes to the exec environment. This adds or removes bodies to/from the environment scene
    // and notifies all registered planning environments about the environment changes
    unsigned int numChanges = environmentChanges_->getNumChanges();
    EnvironmentChangeSharedPtr change = environmentChanges_->getNextChange();
    while (change) {
        //LOGGING("Apply change in " + prefix_ + ", " + std::to_string(threadId_));
        switch (change->getType()) {
        case EnvironmentChangeType::BODY_ADDED: {
            //LOGGING("Apply body added change");
            std::string sdfString = change->as<BodyAddedChange>()->getSDFString();
            SDFEnvironmentParser sdfParser;
            VectorBodyUniquePtr bodies = sdfParser.parseBodiesFromSDFString(sdfString);
            for (size_t i = 0; i != bodies.size(); ++i) {
                if (bodies[i])
                    addBody(std::move(bodies[i]), change->requiresCallback());
            }

            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::MODEL_ADDED: {
            std::string sdfString = change->as<BodyAddedChange>()->getSDFString();
            SDFEnvironmentParser sdfParser;
            VectorBodyUniquePtr bodies = sdfParser.parseBodiesFromSDFString(sdfString);
            for (size_t i = 0; i != bodies.size(); ++i) {
                if (bodies[i])
                    addBody(std::move(bodies[i]), false);
            }

            getGazeboInterface()->addBodyFromSDFString(sdfString);
            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::MODEL_REMOVED: {
            std::string modelName = change->as<ModelRemovedChange>()->getModelName();
            removeModel(modelName);
            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::BODY_REMOVED: {
            std::string removedBodyName = change->as<BodyRemovedChange>()->getBodyName();
            removeBody(removedBodyName, change->requiresCallback());
            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::BODY_POSE_CHANGED: {
            const std::string bodyName = change->as<BodyPoseChange>()->getBodyName();
            //LOGGING("Apply body pose change " + bodyName);
            const geometric::Pose pose = change->as<BodyPoseChange>()->getPose();
            changeBodyPose(bodyName, pose, change->requiresCallback());
            notifyPlanningEnvironments(change);
            break;
        }
        default: {
            WARNING("Change type not recognized");
            break;
        }

        }

        change = environmentChanges_->getNextChange();
    }

#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (numChanges > 0 && viewer) {
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_, true);
    }
#endif
}

void RobotEnvironment::notifyPlanningEnvironments(const EnvironmentChangeSharedPtr& environmentChange)
{
    // Calls onEnvironmentChanged for the registered planning environments.
    // This method is executed from the exec environment
    for (auto & environmentChangedNotificationFunction : onEnvironmentChangedFns_) {
        environmentChangedNotificationFunction(environmentChange);
    }
}

void RobotEnvironment::registerForEnvironmentChanges(RobotEnvironment* planningEnvironment)
{
    for (auto & registeredEnvironment : registeredRobotEnvironments_) {
        if (planningEnvironment == registeredEnvironment) {
            LOGGING("RobotEnvironment already registered for changes");
            return;
        }
    }

    // This is executed in the exec environment and called by the planning environments
    LOGGING("Registering (" +
            planningEnvironment->getPrefix() +
            ", " +
            std::to_string(planningEnvironment->getThreadId()) +
            ") to (" + prefix_ + ", " + std::to_string(threadId_));
    OnEnvironmentChangedFunction onEnvironmentChangedFunction(std::bind(&RobotEnvironment::onEnvironmentChanged,
            planningEnvironment,
            std::placeholders::_1));
    onEnvironmentChangedFns_.push_back(onEnvironmentChangedFunction);
    registeredRobotEnvironments_.push_back(planningEnvironment);
}

bool RobotEnvironment::unregisterForEnvironmentChanges(RobotEnvironment *const planningEnvironment) {
    for (size_t i = 0; i != registeredRobotEnvironments_.size(); ++i) {
        if (registeredRobotEnvironments_[i] == planningEnvironment) {
            LOGGING("Unregistering environment (" +
                    planningEnvironment->getPrefix() +
                    ", " +
                    std::to_string(planningEnvironment->getThreadId()) +
                    ") from (" +
                    prefix_ + ", " + std::to_string(threadId_) + ")");
            registeredRobotEnvironments_.erase(registeredRobotEnvironments_.begin() + i);
            onEnvironmentChangedFns_.erase(onEnvironmentChangedFns_.begin() + i);
            return true;
        }
    }

    return false;
}

void RobotEnvironment::onEnvironmentChanged(const EnvironmentChangeSharedPtr& environmentChange)
{
    // This is executed by the planning environments when they get informed
    // by the exec environment that there was an environment change
    switch (environmentChange->getType()) {
    case EnvironmentChangeType::BODY_ADDED: {
        std::string sdfString = environmentChange->as<BodyAddedChange>()->getSDFString();
        SDFEnvironmentParser sdfParser;
        VectorBodyUniquePtr bodies = sdfParser.parseBodiesFromSDFString(sdfString);
        for (size_t i = 0; i != bodies.size(); ++i) {
            if (bodies[i]) {
                addBody(std::move(bodies[i]), environmentChange->requiresCallback());
                notifyPlanningEnvironments(environmentChange);
            }
        }

        break;
    }
    case EnvironmentChangeType::MODEL_ADDED: {
        std::string sdfString = environmentChange->as<BodyAddedChange>()->getSDFString();
        SDFEnvironmentParser sdfParser;
        VectorBodyUniquePtr bodies = sdfParser.parseBodiesFromSDFString(sdfString);
        for (size_t i = 0; i != bodies.size(); ++i) {
            if (bodies[i])
                addBody(std::move(bodies[i]), false);
        }

        getGazeboInterface()->addBodyFromSDFString(sdfString);
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    case EnvironmentChangeType::MODEL_REMOVED: {
        std::string modelName = environmentChange->as<ModelRemovedChange>()->getModelName();
        removeModel(modelName);
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    case EnvironmentChangeType::BODY_REMOVED: {
        std::string bodyName = environmentChange->as<BodyRemovedChange>()->getBodyName();
        removeBody(bodyName, environmentChange->requiresCallback());
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    case EnvironmentChangeType::BODY_POSE_CHANGED: {
        std::string bodyName = environmentChange->as<BodyPoseChange>()->getBodyName();
        changeBodyPose(bodyName,
                       environmentChange->as<BodyPoseChange>()->getPose(),
                       environmentChange->requiresCallback());
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    default: {
        WARNING("Change type not recognized");
        break;
    }
    }
}

void RobotEnvironment::setRandomEngine(RandomEnginePtr randomEngine)
{
    randomEngine_ = randomEngine;
}

void RobotEnvironment::makeEnvironmentInfo(const bool& interactive)
{
    environmentInfo_ = std::make_shared<oppt::EnvironmentInfo>(interactive);
    environmentInfo_->setScene(scene_);
    robot_->setEnvironmentInfo(environmentInfo_);
    robot_->initEnvironmentCallbacks();
}

void RobotEnvironment::setEnvironmentInfo(std::shared_ptr<oppt::EnvironmentInfo>& environmentInfo)
{
    environmentInfo_ = environmentInfo;
    scene_ = environmentInfo_->getScene();
    robot_->setEnvironmentInfo(environmentInfo_);
    robot_->initEnvironmentCallbacks();
}

EnvironmentInfoSharedPtr RobotEnvironment::getEnvironmentInfo() const {
    return environmentInfo_;
}

oppt::Robot* RobotEnvironment::getRobot() const
{
    return robot_.get();
}

bool RobotEnvironment::loadEnvironment(std::string environment_file, const std::string& robotName, const bool& reset)
{
    // Create an empty scene
    scene_ = std::make_shared<SceneImpl>();

    // Populate it with bodies from the environment file (if it is defined)
    if (environment_file.empty() == false) {        
        SDFEnvironmentParser sdfParser;
        VectorBodyUniquePtr bodies = sdfParser.parseBodiesFromFile(environment_file, robotName);
        scene_->addBodies(bodies);
        if (!robot_) {
            ERROR("RobotEnvironment: Error: Robot has not been initialized yet");
        }

        if (reset and gazeboInterface_)
            gazeboInterface_->reset();
    }
    return true;
}

bool RobotEnvironment::addBody(BodyUniquePtr body, bool executeCallbacks)
{
    environmentInfo_->getScene()->addBody(std::move(body), executeCallbacks);
#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (viewer) {
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_);
    }
#endif
    return true;

}

bool RobotEnvironment::changeBodyPose(const std::string& name, const geometric::Pose& pose, bool executeCallbacks)
{
    bool poseChanged = environmentInfo_->getScene()->changeBodyPose(name, pose, executeCallbacks);
#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (viewer)
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_);
#endif
    return poseChanged;
}

bool RobotEnvironment::removeModel(const std::string &modelName) {
    environmentInfo_->getScene()->removeModel(modelName);
    getGazeboInterface()->removeBody(modelName);
    return true;
}

bool RobotEnvironment::removeBody(std::string bodyName, bool executeCallbacks)
{
    environmentInfo_->getScene()->removeBody(bodyName, executeCallbacks);
    return true;
}

const std::string RobotEnvironment::getWorldFile() const
{
    return worldFile_;
}

std::vector<RobotEnvironment*> RobotEnvironment::getRegisteredRobotEnvironments() const
{
    return registeredRobotEnvironments_;
}

bool RobotEnvironment::setExecutionEnvironment(const bool& executionEnvironment)
{
    isExecutionEnvironment_ = executionEnvironment;
    return true;
}

void RobotEnvironment::setThreadId(const unsigned int& threadId)
{
    threadId_ = threadId;
}

const unsigned int RobotEnvironment::getThreadId() const
{
    return threadId_;
}

const bool RobotEnvironment::isExecutionEnvironment() const
{
    return isExecutionEnvironment_;
}

const SceneSharedPtr RobotEnvironment::getScene() const
{
    return scene_;
}

FloatType RobotEnvironment::getReward(const PropagationResultSharedPtr& propagationResult) const
{
    if (!rewardPlugin_)
        ERROR("No reward plugin loaded");
    return rewardPlugin_->getReward(propagationResult);
}

RewardPlugin *const RobotEnvironment::getRewardPlugin() const {
    return rewardPlugin_.get();
}

void RobotEnvironment::createInitialBeliefPlugin(const std::string& pluginFile) {
    initialBeliefPlugin_ = InitialBeliefPlugin::Create(pluginFile, "initialBeliefPlugin");
    initialBeliefPlugin_->robotEnvironment_ = this;
    initialBeliefPluginFile_ = pluginFile;
}

void RobotEnvironment::createRewardPlugin(const std::string& pluginFile) {
    rewardPlugin_ = RewardPlugin::Create(pluginFile, "rewardPlugin");
    rewardPlugin_->robotEnvironment_ = this;
    rewardPluginFile_ = pluginFile;
}

void RobotEnvironment::createTerminalPlugin(const std::string& pluginFile) {
    terminalPlugin_ = TerminalPlugin::Create(pluginFile, "terminalPlugin");
    terminalPlugin_->robotEnvironment_ = this;
    terminalPluginFile_ = pluginFile;
}

void RobotEnvironment::loadInitialBeliefPlugin(const std::string& optionsFile)
{
    if (initialBeliefPlugin_ == nullptr)
        ERROR("Initial belief plugin hasn't been created yet");
    
    if (!initialBeliefPlugin_->load(optionsFile))
        ERROR("InitialBeliefPlugin '" + initialBeliefPluginFile_ +
              "' couldn't be loaded (the library was found, but the load() method returned false)");    
    initialBeliefOptionsFile_ = optionsFile;
}

void RobotEnvironment::loadRewardPlugin(const std::string& optionsFile)
{
    if (rewardPlugin_ == nullptr)
        ERROR("Initial belief plugin hasn't been created yet");
    
    if (!rewardPlugin_->load(optionsFile))
        ERROR("InitialBeliefPlugin '" + initialBeliefPluginFile_ +
              "' couldn't be loaded (the library was found, but the load() method returned false)");    
    rewardPluginOptionsFile_ = optionsFile;
}

void RobotEnvironment::loadTerminalPlugin(const std::string& optionsFile)
{
    if (terminalPlugin_ == nullptr)
        ERROR("Initial belief plugin hasn't been created yet");
    
    if (!terminalPlugin_->load(optionsFile))
        ERROR("InitialBeliefPlugin '" + initialBeliefPluginFile_ +
              "' couldn't be loaded (the library was found, but the load() method returned false)");    
    terminalPluginOptionsFile_ = optionsFile;
}

bool RobotEnvironment::isTerminal(const PropagationResultSharedPtr& propagationResult) const
{
    return terminalPlugin_->isTerminal(propagationResult);
}

bool RobotEnvironment::isValid(const PropagationResultSharedPtr& propagationResult) const
{
    return terminalPlugin_->isValid(propagationResult)->isValid;
}

RobotStateSharedPtr RobotEnvironment::sampleInitialState() const
{
    if (!initialBeliefPlugin_)
        ERROR("Initial belief plugin not loaded");
    RobotStateSharedPtr initState = initialBeliefPlugin_->sampleAnInitState();
    RobotStateSharedPtr initialStateNormalized =
        robot_->getStateSpace()->normalizeState(initState);
    return initialStateNormalized;
}

bool RobotEnvironment::makeGazeboInterface()
{
    std::string worldFile = getWorldFile();
    std::string robotName = getRobot()->getRobotName();
    std::string prefix = getPrefix();
    unsigned int threadId = getThreadId();
    gazeboInterface_ = std::make_unique<GazeboInterface>(worldFile, robotName, prefix, threadId);
    return true;
}

GazeboInterface* const RobotEnvironment::getGazeboInterface() const
{
    if (!gazeboInterface_) {
        std::string msg = "You're trying to access the GazeboInterface, but the GazeboInterface hasn't been initialized";
        msg += " since you didn't specify the 'planningEnvironmentPath' and 'executionEnvironmentPath' in your problem configuration file";        
        ERROR(msg);
    }
    return gazeboInterface_.get();
}

InitialBeliefPlugin *const RobotEnvironment::getInitialBeliefPlugin() const {
    return initialBeliefPlugin_.get();
}

TerminalPlugin *const RobotEnvironment::getTerminalPlugin() const {
    return terminalPlugin_.get();
}

RobotEnvironmentSharedPtr RobotEnvironment::clone(const unsigned int& uid) {
    if (prefix_ == "exec")
        ERROR("Attempting to clone the robot operational environment");
    std::shared_ptr<RobotEnvironment> env = std::make_shared<RobotEnvironment>();
    env->setRandomEngine(randomEngine_);
    env->setThreadId(uid);
    env->createRobot<oppt::RobotImpl>(worldFile_,
                                      getRobot()->getName(),
                                      configPath_,
                                      prefix_,
                                      uid);
    env->getRobot()->init();    

    env->getRobot()->normalizedSpaces_ = robot_->normalizedSpaces_;
    EnvironmentInfoSharedPtr clonedEnvironmentInfo = environmentInfo_->clone();
    env->setEnvironmentInfo(clonedEnvironmentInfo);
    StateSpaceInfo stateSpaceInfo = robot_->getStateSpace()->getInfo();
    env->getRobot()->makeStateSpace(stateSpaceInfo);
    ObservationSpaceInfo observationSpaceInfo = robot_->getObservationSpace()->getInfo();
    env->getRobot()->makeObservationSpace(observationSpaceInfo);
    ActionSpaceInfo actionSpaceInfo = robot_->getActionSpace()->getInfo();
    env->getRobot()->makeActionSpace(actionSpaceInfo);
    if (!env->getRobot()->getActionSpace()) {
        oppt::ERROR("RobotEnvironment: Robot has no action space!");
    }

    env->initializeEnvironmentCallbacks();

    env->createInitialBeliefPlugin(initialBeliefPluginFile_);
    env->createRewardPlugin(rewardPluginFile_);
    env->createTerminalPlugin(terminalPluginFile_);
    env->getRobot()->createTransitionPlugin(robot_->transitionPluginFile_, env.get());
    env->getRobot()->createObservationPlugin(robot_->observationPluginFile_, env.get()); 

    env->loadInitialBeliefPlugin(initialBeliefOptionsFile_);
    env->loadRewardPlugin(rewardPluginOptionsFile_);
    env->loadTerminalPlugin(terminalPluginOptionsFile_);
    env->getRobot()->loadTransitionPlugin(robot_->transitionPluginOptionsFile_);
    env->getRobot()->loadObservationPlugin(robot_->observationPluginOptionsFile_);

    env->getRobot()->setCollisionsAllowed(robot_->getCollisionsAllowed());
    auto ikSolver = std::move(robot_->getIKSolver()->clone());
    env->getRobot()->setIKSolver(std::move(ikSolver));
    return env;
}

}


