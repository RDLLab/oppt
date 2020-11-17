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
#ifndef ROBOT_INTERFACE_HPP_
#define ROBOT_INTERFACE_HPP_
#include <boost/thread.hpp>
#include <fcl/BV/BV.h>
#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <random>

#include "StateSpace.hpp"
#include "ActionSpace.hpp"
#include "ObservationSpace.hpp"

#include "oppt/opptCore/core.hpp"
#include "RobotState.hpp"
#include "Serializer.hpp"
#include "Metric.hpp"
#include "InverseKinematics/IKSolver.hpp"

using std::cout;
using std::endl;

namespace oppt
{

class TransitionPlugin;
class ObservationPlugin;

class Robot
{

public:
    friend class RobotEnvironment;
    friend class ProblemEnvironment;

    /**
     * Constructor of the robot class
     *
     * @param worldFile The full path to the environment SDF file
     * @param robotName The name of the model in the SDF file that corresponds to the robot
     * @param configFile The full path to the problem configuration file
     * @param prefix A prefix used for several names within the robot class
     * @param threadId The unique ID of the thread this robot class is instantiated in
     */
    Robot(const std::string &worldFile,
          const std::string &robotName,
          const std::string &configFile,
          const std::string &prefix,
          unsigned int& threadId);

    virtual ~Robot();

    /**
     * @brief Forward propagation of a state
     *
     * This method is the central method to perform forward propagations of a robot state.
     * The method takes a PropagationRequestSharedPtr as an input parameter, which contains
     * a state, and action and other information required to propagate the state forward using the action.
     * If the propagationRequest doesn't contain a process error vector, it will be sampled here
     *
     * @param propagationRequest The PropagationRequest which contains all the information required
     * to perform a forward propagation
     * @return A shared pointer to the PropagationResult which contains the next state and other information
     *
     */
    virtual PropagationResultSharedPtr propagateState(PropagationRequestSharedPtr& propagationRequest);

    /**
     * @brief Generating observation given an observationRequest
     *
     * This method is the central method to sample observations given a particular state in the
     * observationRequest parameter. If observationRequest doesn't contain an observation error,
     * it will be sampled here
     *
     * @param observationRequest The ObservationRequest which contains all the information required
     * to sample an observation
     * @return A shared pointer to the ObservationResult which contains the sampled observation for the
     * state that was given in the observationRequest
     */
    virtual ObservationResultSharedPtr makeObservationReport(ObservationRequestSharedPtr& observationRequest) const;

    /**
     * Generates a dirty collision report. 'Dirty' means that the underlying state of the robot
     * is not updated before collision checking is performed
     *
     * @return A shared_ptr to an oppt::CollisionReport
     */
    virtual CollisionReportSharedPtr makeDiscreteCollisionReportDirty() const = 0;

    /**
     * @brief Generates a discrete collision report
     *
     * It uses the underlying oppt::DiscreteCollisionFunction
     * to perform collision checking, which can be changed using the oppt::Robot::setDiscreteCollisionFunction method
     *
     * @param state The RobotState for which collision checking is performed
     * @return A shared pointer to a CollisionReport
     */
    virtual oppt::CollisionReportSharedPtr makeDiscreteCollisionReport(const oppt::RobotStateSharedPtr& state) const;

    /**
     * @brief Generates a continuous collision report
     *
     * Generates a continuous collision report between two states. This is done by
     * linear interpolation between the states an discretizing the resulting linear trajectory.
     * The resolution of the discretization can be controlled with the numIterations parameter
     *
     * @param state1 The first state
     * @param state2 The second state
     * @param numIterations The number of discretization points between the two states
     *
     * @return A shared pointer to a CollisionReport which constains collision information
     */
    virtual oppt::CollisionReportSharedPtr makeContinuousCollisionReport(const oppt::RobotStateSharedPtr& state1,
            const oppt::RobotStateSharedPtr& state2,
            const unsigned int& numIterations = 10) const;

    /**
     * @brief Creates the transition plugin
     *
     * @param pluginFile The file to load the plugin from
     * @param robotEnvironment A pointer to the RobotEnvironment this robot belongs to
     */
    void createTransitionPlugin(const std::string& pluginFile,
                                RobotEnvironment* robotEnvironment);

    /**
     * @brief Creates the observation plugin
     *
     * @param pluginFile The file to load the plugin from
     * @param robotEnvironment A pointer to the RobotEnvironment this robot belongs to
     */
    void createObservationPlugin(const std::string& pluginFile,
                                 RobotEnvironment* robotEnvironment);

    /**
     * @brief Loads the transition plugin. This method calls the oppt::Plugin::load() method
     */
    void loadTransitionPlugin(const std::string& optionsFile);    

    /**
     * @brief Loads the observation plugin. This method calls the oppt::Plugin::load() method
     */
    void loadObservationPlugin(const std::string& optionsFile);

    // ******************** Pure virtual methods **************************
    /**
     * @brief Get the name of the robot
     */
    virtual std::string getName() const;


    /**
     * @brief Constructs the collision objects for the given state
     *
     * @param state The state for which the collision objects are computed for
     */
    virtual void createRobotCollisionObjects(const oppt::RobotStateSharedPtr& state) const = 0;

    /**
     * @brief Get the degrees-of-freedom of the robot
     */
    virtual unsigned int getDOF() const = 0;

    virtual void updateViewer(const oppt::RobotStateSharedPtr& state,
                              const std::vector<oppt::RobotStateSharedPtr>& particles = std::vector<oppt::RobotStateSharedPtr>(),
                              const FloatType& particleOpacity = 1.0,
                              const bool& keepParticles = false,
                              const bool &deleteExistingParticles = false) = 0;

    //****************** End of pure virtual methods ***********************
    /**
     * @brief Get a shared pointer to the underlying StateSpace
     */
    oppt::StateSpaceSharedPtr getStateSpace() const;

    /**
     * @brief Get a shared pointer to the underlying ObservationSpace
     */
    oppt::ObservationSpaceSharedPtr getObservationSpace() const;

    /**
     * @brief Get a shared pointer to the underlying ActionSpace
     */
    oppt::ActionSpaceSharedPtr getActionSpace() const;

    /**
     * @brief Set a custom serializer
     */
    void setSerializer(oppt::SerializerSharedPtr &serializer);

    Serializer* getSerializer() const;

    /**
     * @brief Retuns true if colliding states are not terminal
     */
    bool getCollisionsAllowed() const;

    /**
     * @brief Returns a pointer to the RandomEngine used by this robot
     */
    RandomEnginePtr getRandomEngine() const;

    /**
     * @brief Returns the file this robot was constructed from
     */
    const std::string getRobotName() const;

    FloatType calcLikelihood(const RobotStateSharedPtr &state,
                             const Action *action,
                             const Observation *observation);

    /**
     * @brief Returns true if state, action and observation spaces are normalized
     */
    bool normalizedSpaces() const;

    /**
     * @brief Returns a pointer to the transition plugin
     */
    TransitionPlugin *const getTransitionPlugin() const;

    /**
     * @brief Returns a pointer to the observation plugin
     */
    ObservationPlugin *const getObservationPlugin() const;

    /**
     * @brief Set the oppt::DiscreteCollisionFunction that is used by oppt::Robot::makeDiscreteCollisionReport
     * to perform collision checking
     *
     * @param discreteCollisionFunction The oppt::DiscreteCollisionFunction
     */
    void setDiscreteCollisionFunction(DiscreteCollisionFunction discreteCollisionFunction);

    /**
     * @brief Set the oppt::ContinuousCollisionFunction that is used by oppt::Robot::makeContinuousCollisionReport
     * to perform continuous collision checking
     *
     * @param continuousCollisionFunction The oppt::ContinuousCollisionFunction
     */
    void setContinuousCollisionFunction(ContinuousCollisionFunction continuousCollisionFunction);

    virtual void setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction) = 0;

    ViewerBase *const getViewer() const;

    void setIKSolver(IKSolverUniquePtr ikSolver);

    IKSolver *const getIKSolver() const;

    std::pair<bool, VectorFloat> getIKSolution(const geometric::Pose &pose,
            const unsigned int &numAttempts,
            const VectorFloat &qInit = VectorFloat()) const;

    /**
     * @brief Get a vector of link names that are considered during collision checking
     */
    virtual VectorString getCollisionCheckedLinks() const = 0;

protected:
    std::string robotName_;

    std::string configFile_;

    RandomEnginePtr randomEngine_;

    std::unique_ptr<TransitionPlugin> transitionPlugin_;

    std::unique_ptr<ObservationPlugin> observationPlugin_;

    mutable oppt::StateSpaceSharedPtr stateSpace_;

    mutable oppt::ObservationSpaceSharedPtr observationSpace_;

    mutable oppt::ActionSpaceSharedPtr actionSpace_;

    oppt::EnvironmentInfoSharedPtr environmentInfo_;

    oppt::SerializerSharedPtr serializer_;

    oppt::ViewerSharedPtr viewer_;

    mutable bool collisionsAllowed_;

    std::string prefix_;

    GazeboInterface* gazeboInterface_ = nullptr;

    VectorCollisionObjectUniquePtr robotCollisionObjects_;

    VectorCollisionObjectUniquePtr invariantRobotCollisionObjects_;

    bool normalizedSpaces_ = false;

    DiscreteCollisionFunction discreteCollisionFunction_ = nullptr;

    ContinuousCollisionFunction continuousCollisionFunction_ = nullptr;

    DirtyCollisionFunction dirtyCollisionFunction_ = nullptr;

    IKSolverUniquePtr ikSolver_;

protected:

    virtual void addBodyCallback(Body *const body);

    virtual void removeBodyCallback(std::string name);

    virtual void changeBodyPoseCallback(const std::string& name, const geometric::Pose& poseVec);

    virtual bool init();

    virtual void setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo);

    virtual void initEnvironmentCallbacks();

    virtual void registerOnAddBodyCallback(std::function<void(const std::string&)>& callback);

    virtual void registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback);

    virtual void registerOnPoseChangedCallback(std::function<void(const std::string&, const geometric::Pose&)>& callback);

    virtual void setCollisionsAllowed(const bool& collisionsAllowed) const;

private:
    std::string transitionPluginFile_ = "";
    std::string transitionPluginOptionsFile_ = "";
    std::string observationPluginFile_ = "";
    std::string observationPluginOptionsFile_ = "";

private:
    virtual bool makeActionSpace(ActionSpaceInfo& actionSpaceInfo) = 0;

    virtual bool makeStateSpace(StateSpaceInfo& stateSpaceInfo) = 0;

    virtual bool makeObservationSpace(ObservationSpaceInfo& observationSpaceInfo) = 0;

    void setRandomEngine(RandomEnginePtr randomEngine);

    void setGazeboInterface(GazeboInterface* const gazeboInterface);

    bool initializeViewer_(const std::string &robotName, const std::string &environmentFile, const unsigned int &frameRate);

    virtual void setupViewer(const std::string &robotName, const std::string &environmentFile, const unsigned int &frameRate);

};

}

#endif
