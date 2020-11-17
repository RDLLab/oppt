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

#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

#include "oppt/plugin/Plugin.hpp"

using std::cout;
using std::endl;

namespace oppt
{

Robot::Robot(const std::string &worldFile,
             const std::string &robotName,
             const std::string &configFile,
             const std::string &prefix,
             unsigned int& threadId):
    robotName_(robotName),
    viewer_(nullptr),
    environmentInfo_(nullptr),
    stateSpace_(nullptr),
    actionSpace_(nullptr),
    observationSpace_(nullptr),
    serializer_(nullptr),
    randomEngine_(nullptr),
    configFile_(configFile),
    collisionsAllowed_(false),
    robotCollisionObjects_(),
    prefix_(prefix)
{
    ikSolver_ = std::make_unique<DefaultIKSolver>(randomEngine_.get(), "", "", "");
}

Robot::~Robot()
{
}

bool Robot::init()
{
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
}

void Robot::setRandomEngine(RandomEnginePtr randomEngine)
{
    randomEngine_ = randomEngine;
}

RandomEnginePtr Robot::getRandomEngine() const
{
    return randomEngine_;
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
    propagationResult->errorVector = propagationRequest->errorVector;
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
}

void Robot::addBodyCallback(Body *const body)
{

}

void Robot::removeBodyCallback(std::string name)
{

}

void Robot::changeBodyPoseCallback(const std::string& name, const geometric::Pose& poseVec)
{

}

void Robot::initEnvironmentCallbacks()
{

}

void Robot::registerOnAddBodyCallback(std::function<void(const std::string&)>& callback)
{

}

void Robot::registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback)
{

}

void Robot::registerOnPoseChangedCallback(std::function<void(const std::string&, const geometric::Pose&)>& callback)
{

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

void Robot::setGazeboInterface(GazeboInterface* const gazeboInterface)
{
    gazeboInterface_ = gazeboInterface;
}

bool Robot::normalizedSpaces() const
{
    return normalizedSpaces_;
}

TransitionPlugin* const Robot::getTransitionPlugin() const
{
    return transitionPlugin_.get();
}

ObservationPlugin* const Robot::getObservationPlugin() const
{
    return observationPlugin_.get();
}

void Robot::setDiscreteCollisionFunction(DiscreteCollisionFunction discreteCollisionFunction)
{
    discreteCollisionFunction_ = discreteCollisionFunction;
}

void Robot::setContinuousCollisionFunction(ContinuousCollisionFunction continuousCollisionFunction)
{
    continuousCollisionFunction_ = continuousCollisionFunction;
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


}
