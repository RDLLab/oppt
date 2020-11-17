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
#ifndef __GAZEBO_INTERFACE_HPP__
#define __GAZEBO_INTERFACE_HPP__
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <queue>
#include <sdf/parser.hh>
#include "sensors/SensorInterface.hpp"
#include <gazebo/msgs/msgs.hh>
#include "gazebo/World.hpp"
#include "ServerInterface.hpp"

namespace gazebo
{
namespace physics
{
class ODELink;
}
}


namespace oppt
{

struct ContactPoint {
    std::string contactBody1;

    std::string contactBody2;

    VectorFloat contactPointWorldPosition;

};

struct ContactInformation {
public:
    ContactInformation() {}

    std::vector<std::unique_ptr<ContactPoint>> contactPoints;

};

/** A request to propagate the physics engine forward */
struct GazeboPropagationRequest {
    /** @brief The current state vector */
    VectorFloat currentStateVec;

    /** @brief The action vector */
    VectorFloat actionVec;

    /** @brief The current GazeboWorldState */
    GazeboWorldStatePtr currentWorldState;

    /** @brief The duration (in seconds) a control has to be applied for */
    FloatType duration;

    /** @brief Enable collision checking */
    bool enableCollision = true;

    /** @brief If collisions are not allowed, the forward propagation will stop when a collision occured */
    bool allowCollisions = false;
};

/** The result of a forward propagation of the physics engine */
struct GazeboPropagationResult {

    /** @brief contains the propagated state values */
    VectorFloat nextStateVec;

    /** @brief The propagated GazeboWorldState */
    GazeboWorldStatePtr nextWorldState;

    std::vector<VectorFloat> subStates;

    /** @brief oppt::CollisionReportSharedPtr to a oppt::CollisionReport */
    CollisionReportSharedPtr collisionReport = nullptr;

    std::unique_ptr<ContactInformation> contactInformation = nullptr;
};

/** @brief std::unique_ptr to GazeboPropagationResult*/
typedef std::unique_ptr<GazeboPropagationResult> GazeboPropagationResultUniquePtr;

/** @brief A request to obtain sensor data */
struct GazeboObservationRequest {
    /** @brief The state vector for which the sensor data has to be obtained for */
    VectorFloat currentStateVec;

    /** @brief The current GazeboWorldState for which the sensor data has to be obtained for */
    GazeboWorldStatePtr currentWorldState;
};

/** @brief The result of a GazeboObservationRequest */
struct GazeboObservationResult {
    /** @brief Vector containing the sensor values */
    VectorFloat observationVec;
};

/** @brief std::unique_ptr to GazeboObservationResult */
typedef std::unique_ptr<GazeboObservationResult> GazeboObservationResultUniquePtr;

struct GazeboObservationReport {
    VectorFloat currentStateVec;

    GazeboWorldStatePtr currentWorldState;

    VectorFloat observationVec;
};

/**
 * A wrapper structure for vectors of bounding boxes
 */
struct BoundingBoxes {
    /**
     * @brief The names of the links for which the bounding boxes are contained inside boxes
     */
    VectorString linkNames;

    /**
     * @brief A vector containing the bounding box dimensions
     */
    std::vector<VectorFloat> boxes;

    /**
     * @brief A vector containing the world poses of the bounding boxes
     */
    std::vector<geometric::Pose> worldPoses;
};

typedef std::shared_ptr<std::queue<std::vector<gazebo::physics::JointPtr>>> JointQueuePtr;
typedef std::unordered_map<std::string, gazebo::physics::JointPtr> JointMap;
typedef std::unordered_map<std::string, JointMap> WorldJointMap;
typedef std::unordered_map<std::string, gazebo::physics::LinkPtr> LinkMap;
typedef std::unordered_map<std::string, gazebo::physics::ODELink*> ODELinkMap;
typedef std::unordered_map<std::string, LinkMap> WorldLinkMap;
typedef std::unordered_map<std::string, gazebo::transport::SubscriberPtr> WorldSubscriberMap;
typedef std::unordered_map<std::string, ModelPtr> WorldModelMap;
typedef std::queue<std::pair<std::string, std::string>> BodyEventQueue;
typedef std::function<void(const VectorFloat&)> SetStateFunction;
typedef std::function<void(VectorFloat&)> GetStateFunction;
typedef std::function<void(const VectorFloat&)> ApplyActionFunction;
typedef std::function<void(VectorFloat&)> GetObservationFunction;
typedef std::vector<SetStateFunction> SetStateFunctions;
typedef std::vector<GetStateFunction> GetStateFunctions;
typedef std::vector<ApplyActionFunction> ApplyActionFunctions;
typedef std::vector<GetObservationFunction> GetObservationFunctions;
typedef std::unordered_map<std::string, ignition::math::Pose3d> PoseMap;
typedef std::unordered_map<std::string, std::string> WorldBodyMap;

class GazeboSubscriber;

/**
 * The GazeboInterface class
 */
class GazeboInterface
{
public:
    friend class RobotEnvironment;
    friend class Robot;
    friend class RobotImpl;
    friend class GazeboSubscriber;

    /**
     * Constructor of GazeboInterface
     *
     * @param gazebWorldFile The absolute path the SDF environment model file for which this GazeboInterface is instantiated
     * @param robotName The of the model in the SDF environment file that corresponds to the robot
     * @param prefix Prefix used internally for names
     */
    GazeboInterface(std::string& gazeboWorldFile,
                    std::string& robotName,
                    std::string& prefix,
                    unsigned int threadId = 0);

    ~GazeboInterface();

    /**
     * @brief Returns a pointer to the underlying World
     */
    WorldPtr getWorld() const;

    /**
     * @brief Resets the GazeboInterface and the underlying physics engine,
     * reverting all changes made to the environment during runtime
     */
    void reset();

    /**
     * Step the physics engine forward in time
     *
     * @param propagationReport A GazeboPropagationReport that contains information on how to
     * step the physics engine forward
     */
    GazeboPropagationResultUniquePtr doPropagation(const GazeboPropagationRequest* const propagationRequest);

    /**
     * @brief Obtain an observation from the sensor data, given a state vector.
     *
     * @param observationRequest GazeboObservationRequest containing the state vector and the GazeboWorldState for which
     * the observation has to be obtained for
     *
     * @return A GazeboObservationResultUniquePtr containing the observation vector
     */
    GazeboObservationResultUniquePtr makeObservationReport(const GazeboObservationRequest* const observationRequest);

    /**
     * @brief Constructs an initial world state
     */
    void makeInitialWorldState(const VectorFloat& initialStateVec, const bool &fullReset = true);

    /**
     * @brief Returns a pointer to the constructed initial world state
     *
     * @param setWorldToInitialWorldState If true (default), the current Gazebo world state will be set to the
     * intial world state
     *
     * @returns A oppt::GazeboWorldStatePtr, representing the initial world state
     */
    GazeboWorldStatePtr getInitialWorldState(const bool &setWorldToInitialWorldState = true);

    void getRobotBoundingBoxes(const VectorFloat& stateVec, GazeboWorldStatePtr& worldState, BoundingBoxes* boundingBoxes);

    void getRobotBoundingBoxesDirty(BoundingBoxes* boundingBoxes);

    /**
     * @brief Get the position of a link in world coordinates given a state vector and a GazeboWorldState
     *
     * @param robotState The state of the robot for which the link position is computed for
     * @param worldState The world state for which the link position is computed for
     * @param linkName The name of the link for which the position is computed forw
     *
     * @returns A VectorFloat containing the x,y,z position of the link
     */
    VectorFloat getLinkPosition(const VectorFloat& robotState,
                                const GazeboWorldStatePtr& worldState,
                                const std::string& linkName);

    /**
     * @brief Get the pose of a link in world coordinates given a state vector and a GazeboWorldState
     */
    geometric::Pose getLinkPose(const VectorFloat& robotState,
                                const GazeboWorldStatePtr& worldState,
                                const std::string& linkName);

    std::pair<VectorString, std::vector<geometric::Pose>> getLinkPosesDirty() const;

    /**
     * @brief Get the poses of the center of gravity of the links with names provided in 'linkNames'
     * @param robotState The current state as a vector
     * @param worldState The oppt::GazeboWorldStatePtr that belongs to the state vector
     * @param linkNames oppt::VectorString of the names of the links for which the poses are acquired
     */
    std::vector<geometric::Pose> getLinksCOGPose(const VectorFloat& robotState,
            const GazeboWorldStatePtr& worldState,
            const VectorString& linkNames);

    std::vector<geometric::Pose> getLinksCOGPoseDirty(const VectorString& linkNames);

    std::vector<geometric::Pose> getLinksCollisionWorldPoses(const RobotStateSharedPtr& robotState,
            const VectorString& linkNames);

    std::vector<geometric::Pose> getLinksCollisionWorldPosesDirty(const VectorString& linkNames);

    /**
     * @brief Gets the name of the robot
     */
    std::string getRobotName() const;

    /**
     * @brief Gets the poses in world coordinates of the visual elements of the robot.
     *
     * @param state The state vector for which the poses are computed
     * @param visualNames The names of the visual elements for which the poses are computed
     * @param visualPoses The vector in which the poses are stored in
     */
    void getRobotVisualPoses(VectorFloat& state,
                             const VectorString& visualNames,
                             std::vector<geometric::Pose>& visualPoses,
                             const GazeboWorldStatePtr &worldState = nullptr);

    /**
     * @brief Sets a custom CollisionFunction that is being used when stepping the
     * world forward
     *
     * @param collisionFunction The CollisionFunction to use
     */
    void setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction);

    /**
     * @brief Set the maximum step size of the physics engine
     */
    void setMaxStepSize(const FloatType &stepSize);

    /**
     * @brief Returns a VectorString containing the names of all the joints in the environment
     */
    const VectorString getJointNames() const;

    /**
     * @brief Returns a VectorString containing the names of all the joints of the robot
     */
    const VectorString getRobotJointNames() const;

    /**
     * @brief Returns a VectorString containing the names of all the links the robot consists of
     */
    const VectorString getRobotLinkNames() const;

    /**
     * @brief Returns a VectorString containing the names of all the links the robot and the world consist of
     */
    const VectorString getLinkNames() const;

    /**
     * @brief Sets the world state
     *
     * @param currentWorldState A pointer to the GazeboWorldState
     *
     * @param updateScene Determines if the scene used for collision checking should reflect the new world state
     */
    void setWorldState(const GazeboWorldState *currentWorldState, const bool& applyDefPoseChanges = false, const bool& updateScene=false);

    /**
     * @brief Sets the state of the robot from the given state vector
     *
     * @param stateVector The state vector
     */
    void setStateVector(const VectorFloat& stateVector);

    /**
     * @brief Gets the current GazeboWorldState
     *
     * @param generateNewWorldState True iff a new world state should be constructed. If this parameter is false,
     * the last constructed world state will be returned. Default is false
     * @returns A pointer to the current GazeboWorldState
     */
    const GazeboWorldStatePtr getWorldState(const bool &generateNewWorldState = false);

    /**
     * @brief Set the soft limit threshold
     *
     * @param softLimitThreshold The soft limit threshold
     */
    void setSoftLimitThreshold(const FloatType& softLimitThreshold);

    /**
     * Get a vector of all links that are present int the world
     */
    std::vector<gazebo::physics::Link *> getLinks() const;

    /**
     * Get a vector of all joints that are present in the world
     */
    std::vector<gazebo::physics::Joint *> getJoints() const;

    /**
     * Set a callback function that is being called before an update of the physics engine
     */
    void setBeforePhysicsUpdateFn(std::function<void()> beforePhysicsUpdateFn);

    /**
     * Set a callback function that is being called after an update of the physics engine
     */
    void setAfterPhysicsUpdateFn(std::function<void()> afterPhysicsUpdateFn);

    /**
     * Set the velocity of a joint recursively
     */
    void setJointVelocityRecursive(gazebo::physics::Joint *joint, const FloatType& velocity) const;

    /**
     * Turn on/off snapping of velocities
     */
    void setSnapVelocities(const bool &snap);

    /**
     * @brief Don't notify the solver about environment changes
     */
    void ignoreEnvironmentChanges();

private:
    void makeInitialWorldModelMap();

    void checkSDFValidity(const std::string& worldFile, const std::string& robotName);    

    LinkPtr findRootLink(const ModelPtr& model);

    void fakeStep();

    void snapVelocityRecursive(const bool& verbose = false);

    void snapVelocityRecursiveHelper(gazebo::physics::JointPtr& joint);

    void setLinkPoseRecursive(LinkPtr& link, GZPose& pose, GZPose& childLinkPose);

    ModelPtr findRobotModel(std::string& robotName);

    ModelPtr findRobotModelImpl(const ModelPtr& model, std::string& robotName);

    void collectAllLinks(WorldPtr& world, std::vector<gazebo::physics::LinkPtr>& links);

    void collectAllLinksImpl(const ModelPtr& model, std::vector<gazebo::physics::LinkPtr>& links);

    void setStateManuallyNew(const VectorFloat& currentStateVec);

    VectorFloat getState_();

    //void setJointPositions(const VectorString &joints, const VectorFloat &angles);

    void removeBodiesFromWorld();

    void onAddBody(const std::string& str);

    void onRemoveBody(const std::string& str);

    void processBodyEvents(std::shared_ptr<BodyEventQueue>& bodyEventQueue);

    std::unordered_map<std::string, VectorString> generateRedundancyMap(const VectorString& joints,
            const std::vector<VectorString>& redundantJoints) const;

    std::string getUnscopedName(const std::string& name) const;

    void printWorldState();

    void printWorldState(GazeboWorldStatePtr& worldState);

    void applyDeferredPoseChanges();

    void setStateSpaceInformation(const StateSpaceInformationPtr& stateSpaceInformation);

    void setActionSpaceInformation(const ActionSpaceInformationPtr& actionSpaceInformation);

    void setObservationSpaceInformation(const ObservationSpaceInformationPtr& observationSpaceInformation);

    void registerOnAddBodyCallback(std::function<void(const std::string&)>& callback);

    void registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback);

    void registerOnPoseChangedCallback(std::function<void(const std::string&, const geometric::Pose&)>& callback);

    void setCollisionCheckedLinks(const VectorString& collisionCheckedLinks);

    void getStateLimits(VectorFloat& lowerStateLimits, VectorFloat& upperStateLimits);

    void getObservationLimits(VectorFloat& lowerObservationLimits, VectorFloat& upperObservationLimits, const bool &ignoreObservationLimits);

    void getActionLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits);

    void addBodyFromSDFString(std::string& sdfString);

    void removeBody(std::string name);

    void changeModelPose(const std::string& name, const geometric::Pose& pose);

    void setInteractive(const bool& interactive);

    std::string getWorldName(std::string& gazeboWorldFile,
                             const unsigned int& threadId);

    void initWorldFromFile(const std::string& gazeboWorldFile,
                           const unsigned int& threadId);

    WorldPtr loadWorld(const std::string& gazeboWorldFile);

    void initRobotModel(const std::string& worldName,
                        ModelPtr& model,
                        WorldLinkMap& worldRobotLinkMap);

    void initWorldLinkMap(WorldLinkMap& worldLinkMap);

    void initWorldJointMap(WorldJointMap &worldJointMap);

    void setObservationSpaceDimension(const unsigned int& dimension);

    GazeboWorldStatePtr makeWorldState();

    void initRootJoints();

    void makeJointVectorHelper_(const VectorString &jointNames,
                                const std::unordered_map<std::string, VectorString> &redundancyMap,
                                std::vector<JointPtr> &jointVector,
                                std::vector<std::vector<JointPtr>> &redundantJoints) const;

    void makeLinkVectorHelper_(const VectorString &linkNames, std::vector<LinkPtr> &links) const;

    void setUpdateSceneFn(std::function<void()> updateSceneFn);

private:
    ServerInterface serverInterface_;

    StateSpaceInformationPtr stateSpaceInformation_;

    ActionSpaceInformationPtr actionSpaceInformation_;

    ObservationSpaceInformationPtr observationSpaceInformation_;

    DirtyCollisionFunction collisionFunction_;

    GazeboWorldStatePtr initialSDFWorldState_ = nullptr;

    GazeboWorldStatePtr initialWorldState_ = nullptr;

    GazeboWorldStatePtr lastConstructedWorldState_ = nullptr;

    WorldBodyMap initialWorldModelMap_;

    std::string mainRobotName_;

    WorldSubscriberMap contactSubs_;

    WorldModelMap robots_;

    WorldJointMap worldJointMap_;

    WorldLinkMap worldRobotLinkMap_;

    WorldLinkMap worldLinkMap_;

    boost::thread clientThread_;

    unsigned int stateSpaceDimension_;

    boost::mutex mtx_;

    std::function<void(const std::string&)> addBodyCallback_;

    std::function<void()> updateSceneFn_;

    std::shared_ptr<BodyEventQueue> bodyEventQueue_;

    boost::thread* processBodiesEventsThread_;

    bool finishProcessingBodies_;

    gazebo::event::ConnectionPtr addEntityConnection_;

    unsigned int threadId_;

    std::unique_ptr<SensorInterface> sensorInterface_ = nullptr;

    SetStateFunctions setStateFunctions_;

    GetStateFunctions getStateFunctions_;

    ApplyActionFunctions applyActionFunctions_;

    GetObservationFunctions getObservationFunctions_;

    VectorString collisionCheckedLinks_;

    std::vector<std::pair<const std::string, const geometric::Pose>> deferredPoseChanges_;

    bool worldRunning_ = false;

    std::string prefix_;

    bool saveUserData_;

    gazebo::common::Time lastSimTime_;

    std::unique_ptr<GazeboSubscriber> gazeboSubscriber_;

    std::vector<LinkPtr> cachedCollisionLinks_;

    std::unordered_map<std::string, JointPtr> rootJoints_;

    unsigned int observationSpaceDimension_ = 0;

    std::function<void()> beforePhysicsUpdateFn_ = nullptr;

    std::function<void()> afterPhysicsUpdateFn_ = nullptr;

    CumulativeAnglesVec initialCumulativeAngles_;

    std::unique_ptr<World> world_ = nullptr;

    bool snapVelocities_ = true;

    bool requiresPhysics_ = false;

    bool ignoringEnvironmentChanges_ = false;
};
}

#endif
