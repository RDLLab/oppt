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
#ifndef __OPPT_TYPEDEFS_HPP__
#define __OPPT_TYPEDEFS_HPP__
#include "includes.hpp"
#include "oppt/defs.hpp"

namespace gazebo {
namespace physics {
class WorldState;
}
}

namespace oppt
{

/** @brief A std::vector of oppt::FloatType */
typedef std::vector<FloatType> VectorFloat;
/** @brief A std::vector of strings */
typedef std::vector<std::string> VectorString;
/** @brief A std::vector of unsigned integers */
typedef std::vector<unsigned int> VectorUInt;
/** @brief A std::vector of signed integers */
typedef std::vector<int> VectorInt;

class SpaceInfo;
typedef std::shared_ptr<SpaceInfo> SpaceInfoSharedPtr;

class SpaceElement;
typedef std::shared_ptr<SpaceElement> SpaceElementSharedPtr;

class SpaceBounds;
typedef std::shared_ptr<SpaceBounds> SpaceBoundsSharedPtr;

#ifdef GZ_GT_7
typedef ignition::math::Pose3d GZPose;
typedef ignition::math::Vector3<FloatType> GZVector3;
typedef ignition::math::Angle GZAngle;
typedef ignition::math::Quaternion<FloatType> GZQuaternion;
typedef ignition::math::Box GZBox;
#else
typedef gazebo::math::Pose GZPose;
typedef gazebo::math::Vector3 GZVector3;
typedef gazebo::math::Angle GZAngle;
typedef gazebo::math::Quaternion GZQuaternion;
typedef gazebo::math::Box GZBox;
#endif

/** @brief Pointer to a Gazebo simulated world */
typedef gazebo::physics::WorldPtr WorldPtr;
typedef gazebo::physics::ModelPtr ModelPtr;
typedef gazebo::physics::LinkPtr LinkPtr;
typedef gazebo::physics::JointPtr JointPtr;

/** @brief std::shared_ptr to a gazebo::physics::WorldState */
typedef std::shared_ptr<gazebo::physics::WorldState> WorldStateSharedPtr;

/** @brief std::unique_ptr to a gazebo::physics::WorldState */
typedef std::unique_ptr<gazebo::physics::WorldState> WorldStateUniquePtr;

/** @brief vector containing the cumulative angles per joint */
typedef std::vector<std::pair<std::string, VectorFloat>> CumulativeAnglesVec;

class GazeboWorldState;
typedef std::shared_ptr<GazeboWorldState> GazeboWorldStatePtr;

typedef std::default_random_engine RandomEngine;
typedef std::shared_ptr<RandomEngine> RandomEnginePtr;

class GazeboInterface;
typedef std::shared_ptr<GazeboInterface> GazeboInterfaceSharedPtr;

struct PropagationRequest;
struct PropagationResult;

/** std::shared_ptr to PropagationRequest */
typedef std::shared_ptr<PropagationRequest> PropagationRequestSharedPtr;

/** std::shared_ptr to PropagationResult */
typedef std::shared_ptr<PropagationResult> PropagationResultSharedPtr;

struct ObservationRequest;
struct ObservationResult;

/** std::shared_ptr to ObservationRequest */
typedef std::shared_ptr<ObservationRequest> ObservationRequestSharedPtr;

/** std::shared_ptr to ObservationResult */
typedef std::shared_ptr<ObservationResult> ObservationResultSharedPtr;

class Body;
/** @brief std::shared_ptr to oppt::Body */
typedef std::shared_ptr<oppt::Body> BodySharedPtr;
/** @brief Vector of oppt::BodySharedPtr */
typedef std::vector<BodySharedPtr> VectorBodySharedPtr;
/** @brief std::unique_ptr to oppt::Body */
typedef std::unique_ptr<oppt::Body> BodyUniquePtr;
/** @brief Vector of oppt::BodyUniquePtr */
typedef std::vector<BodyUniquePtr> VectorBodyUniquePtr;
/** @brief Vector of pointers oppt::Body */
typedef std::vector<Body*> VectorBodyPtr;

class Scene;
typedef std::shared_ptr<oppt::Scene> SceneSharedPtr;

class ViewerBase;
typedef std::shared_ptr<oppt::ViewerBase> ViewerSharedPtr;

struct EnvironmentInfo;
typedef std::shared_ptr<oppt::EnvironmentInfo> EnvironmentInfoSharedPtr;
typedef std::unique_ptr<oppt::EnvironmentInfo> EnvironmentInfoUniquePtr;

namespace geometric
{
class Geometry;
}

/** @brief std::shared_ptr to geometric::Geometry */
typedef std::shared_ptr<geometric::Geometry> GeometrySharedPtr;
/** @brief std::unique_ptr to geometric::Geometry */
typedef std::unique_ptr<geometric::Geometry> GeometryUniquePtr;

/** @brief A vector of oppt::geometric::GeometrySharedPtr*/
typedef std::vector<GeometrySharedPtr> VectorGeometrySharedPtr;

/** @brief A vector of oppt::geometric::GeometryUniquePtr*/
typedef std::vector<GeometryUniquePtr> VectorGeometryUniquePtr;

/** @brief A vector of oppt::geometric::Geometry pointers*/
typedef std::vector<const geometric::Geometry *> VectorGeometryPtr;

typedef fcl::CollisionGeometry FCLCollisionGeometry;
typedef fcl::CollisionObject FCLCollisionObject;
/** @brief std::unique_ptr to oppt::FCLCollisionGeometry */
typedef std::unique_ptr<FCLCollisionGeometry> FCLCollisionGeometryUniquePtr;

// Use pointer type depending on FCL version to ensure backwards compatibility
#ifdef FCL_GT_0_4
/** @brief std::shared_ptr to a fcl::CollisionGeometry */
typedef std::shared_ptr<FCLCollisionGeometry> FCLCollisionGeometrySharedPtr;
/** @brief std::shared_ptr to a fcl::CollisionObject */
//typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectSharedPtr;
#else
/** @brief boost::shared_ptr to a fcl::CollisionGeometry */
typedef boost::shared_ptr<FCLCollisionGeometry> FCLCollisionGeometrySharedPtr;
/** @brief boost::shared_ptr to a fcl::CollisionObject */
//typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectSharedPtr;
#endif
/** @brief std::unique_ptr to fcl::CollisionObject */
typedef std::unique_ptr<fcl::CollisionObject> FCLCollisionObjectUniquePtr;

class EnvironmentChange;
/** @brief std::shared_ptr to oppt::EnvironmentChange */
typedef std::shared_ptr<EnvironmentChange> EnvironmentChangeSharedPtr;
/** @brief std::unique_ptr to oppt::EnvironmentChange */
typedef std::unique_ptr<EnvironmentChange> EnvironmentChangeUniquePtr;
/** @brief Vector of oppt::EnvironmentChangeSharedPtr */
typedef std::vector<EnvironmentChangeSharedPtr> VectorEnvironmentChanges;

class CollisionRequest;
/** @brief std::shared_ptr to oppt::CollisionRequest */
typedef std::shared_ptr<CollisionRequest> CollisionRequestSharedPtr;

class CollisionReport;
/** @brief std::shared_ptr to oppt::CollisionReport */
typedef std::shared_ptr<CollisionReport> CollisionReportSharedPtr;

class Robot;
/** @brief std::shared_ptr to oppt::Robot */
typedef std::shared_ptr<oppt::Robot> RobotSharedPtr;

class RobotState;
class VectorState;

/** @brief std::shared_ptr to RobotState */
typedef std::shared_ptr<oppt::RobotState> RobotStateSharedPtr;
typedef std::shared_ptr<const oppt::RobotState> ConstRobotStateSharedPtr;

/** std::vector of RobotStateSharedPtr */
typedef std::vector<RobotStateSharedPtr> VectorRobotStatePtr;

class RobotEnvironment;

/** @brief A std::shared_ptr to a RobotEnvironment object*/
typedef std::shared_ptr<oppt::RobotEnvironment> RobotEnvironmentSharedPtr;

class Action;
class VectorAction;
/** @brief std::shared_ptr to a oppt::Action */
typedef std::shared_ptr<oppt::Action> ActionSharedPtr;
/** @brief std::unique_ptr to a oppt::Action */
typedef std::unique_ptr<oppt::Action> ActionUniquePtr;
/** @brief A vector of oppt::ActionSharedPtr */
typedef std::vector<ActionSharedPtr> VectorActionSharedPtr;
/** @brief A vector of oppt::ActionUniquePtr */
typedef std::unique_ptr<oppt::VectorAction> VectorActionUniquePtr;

class Observation;
class VectorObservation;
/** @brief std::shared_ptr to a oppt::Observation */
typedef std::shared_ptr<oppt::Observation> ObservationSharedPtr;
/** @brief std::unique_ptr to a oppt::Observation */
typedef std::unique_ptr<oppt::Observation> ObservationUniquePtr;
/** @brief A vector of oppt::ObservationSharedPtr */
typedef std::vector<ObservationSharedPtr> VectorObservationSharedPtr;
/** @brief A vector of oppt::ObservationUniquePtr */
typedef std::unique_ptr<oppt::VectorObservation> VectorObservationUniquePtr;

class ActionSpace;
/** @brief std::shared_ptr to a oppt::ActionSpace */
typedef std::shared_ptr<oppt::ActionSpace> ActionSpaceSharedPtr;
class StateSpace;
/** @brief std::shared_ptr to a oppt::StateSpace */
typedef std::shared_ptr<oppt::StateSpace> StateSpaceSharedPtr;
class ObservationSpace;
/** @brief std::shared_ptr to a oppt::ObservationSpace */
typedef std::shared_ptr<oppt::ObservationSpace> ObservationSpaceSharedPtr;

class LimitsContainer;
typedef std::shared_ptr<LimitsContainer> LimitsContainerSharedPtr;

class ActionLimits;
typedef std::shared_ptr<ActionLimits> ActionLimitsSharedPtr;

class StateLimits;
typedef std::shared_ptr<StateLimits> StateLimitsSharedPtr;

class ObservationLimits;
typedef std::shared_ptr<ObservationLimits> ObservationLimitsSharedPtr;

class HeuristicInfo;
typedef std::shared_ptr<oppt::HeuristicInfo> HeuristicInfoSharedPtr;

class HeuristicFunction;
typedef std::shared_ptr<oppt::HeuristicFunction> HeuristicFunctionSharedPtr;

class Belief;
typedef std::shared_ptr<oppt::Belief> BeliefSharedPtr;

class Particle;
/** @brief std::shared_ptr to oppt::Particle */
typedef std::shared_ptr<oppt::Particle> ParticleSharedPtr;

class Serializer;
typedef std::unique_ptr<oppt::Serializer> SerializerUniquePtr;
typedef std::shared_ptr<oppt::Serializer> SerializerSharedPtr;

class Trajectory;
/** @brief std::shared_ptr to a oppt::Trajectory */
typedef std::shared_ptr<oppt::Trajectory> TrajectorySharedPtr;

/** @brief std::unique_ptr to a oppt::Trajectory */
typedef std::unique_ptr<oppt::Trajectory> TrajectoryUniquePtr;

class Goal;
/** @brief std::shared_ptr to a oppt::Goal */
typedef std::shared_ptr<oppt::Goal> GoalSharedPtr;

struct StateSpaceInformation;
/** @brief std::shared_ptr to oppt::StateSpaceInformation */
typedef std::shared_ptr<StateSpaceInformation> StateSpaceInformationPtr;

struct ActionSpaceInformation;
/** @brief std::shared_ptr to oppt::ActionSpaceInformation */
typedef std::shared_ptr<ActionSpaceInformation> ActionSpaceInformationPtr;

struct ObservationSpaceInformation;
/** @brief std::shared_ptr to oppt::ObservationSpaceInformation */
typedef std::shared_ptr<ObservationSpaceInformation> ObservationSpaceInformationPtr;

typedef std::function < FloatType(const RobotStateSharedPtr& state1,
                                  const RobotStateSharedPtr& state2) > DistanceMetric;

class OpptCollisionObject;
//** @brief std::unique_ptr to oppt::OpptCollisionObject */
typedef std::unique_ptr<OpptCollisionObject> OpptCollisionObjectUniquePtr;
/** @brief Vector of oppt::OpptCollisionObjectUniquePtr */
typedef std::vector<OpptCollisionObjectUniquePtr> VectorCollisionObjectUniquePtr;
/** @brief Vector of pointers to oppt::OpptCollisionObject */
typedef std::vector<OpptCollisionObject*> VectorCollisionObjectPtr;

struct Contact;
/** @brief std::unique_ptr to oppt::Contact */
typedef std::unique_ptr<Contact> ContactUniquePtr;
/** @brief Vector of oppt::ContactUniquePtr */
typedef std::vector<ContactUniquePtr> VectorContacts;

class Filter;
/** @brief std::shared_ptr to a oppt::Filter */
typedef std::shared_ptr<Filter> FilterPtr;

class FilterRequest;
class FilterResult;
/** @brief std::unique_ptr to oppt::FilterRequest*/
typedef std::unique_ptr<FilterRequest> FilterRequestPtr;
/** @brief std::unique_ptr to oppt::FilterResult*/
typedef std::unique_ptr<FilterResult> FilterResultPtr;

/** @brief Definition of a collision function that takes as a input a state and returns a
 * pointer to an oppt::CollisionReport
 */
typedef std::function<oppt::CollisionReportSharedPtr(const RobotStateSharedPtr&)> DiscreteCollisionFunction;

/** @brief Definition of a continuous collision function. Performs collision checking between two states */
typedef std::function < oppt::CollisionReportSharedPtr(const RobotStateSharedPtr&,
        const RobotStateSharedPtr&,
        const unsigned int&) > ContinuousCollisionFunction;

/** @brief A dirty collision function doesn't update the underlying state */
typedef std::function<oppt::CollisionReportSharedPtr(void *const data)> DirtyCollisionFunction;

// FCL stuff
typedef fcl::Transform3f FCLTransform3f;

typedef std::vector<FCLTransform3f> VectorFCLTransform3f;

// Eigen stuff
typedef Eigen::Matrix<FloatType, Eigen::Dynamic, 1> Vectordf;

/** @brief A dynamic-sized Eigen::Matrix of oppt::FloatType*/
typedef Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic> Matrixdf;

/** @brief A 3x3 Eigen::Matrix of oppt::FloatType*/
typedef Eigen::Matrix<FloatType, 3, 3> Matrix3f;

/** @brief A 4x4 Eigen::Matrix of oppt::FloatType*/
typedef Eigen::Matrix<FloatType, 4, 4> Matrix4f;

/** @brief A 3x1 Eigen::Matrix of oppt::FloatType*/
typedef Eigen::Matrix<FloatType, 3, 1> Vector3f;

/** @brief A Eigen::AngleAxis of oppt::FloatType*/
typedef Eigen::AngleAxis<FloatType> AngleAxisf;

/** @brief A Eigen::Transform of oppt::FloatType*/
typedef Eigen::Transform<FloatType, 3, Eigen::AffineCompact> Transform3f;

/** @brief A Eigen::Quaternion of oppt::FloatType*/
typedef Eigen::Quaternion<FloatType> Quaternionf;

}

#endif
