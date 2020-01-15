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
#include "OpptODEPhysics.hpp"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsFactory.hh>
#include "oppt/opptCore/core.hpp"
#include "OpptODELink.hpp"
#include "OpptODEJoint.hpp"

#include <gazebo/physics/ode/ODEHingeJoint.hh>
#include <gazebo/physics/ode/ODEHinge2Joint.hh>
#include <gazebo/physics/ode/ODESliderJoint.hh>
#include <gazebo/physics/ode/ODEScrewJoint.hh>
#include <gazebo/physics/ode/ODEGearboxJoint.hh>
#include <gazebo/physics/ode/ODEFixedJoint.hh>
#include <gazebo/physics/ode/ODEUniversalJoint.hh>
#include <gazebo/physics/ode/ODEBallJoint.hh>

#include <gazebo/ode/common.h>

using namespace gazebo;
using namespace physics;



OpptODEPhysics::OpptODEPhysics(WorldPtr _world):
    ODEPhysics(_world)
{

}

OpptODEPhysics::~OpptODEPhysics() {   

}

void OpptODEPhysics::UpdateCollision()
{
    if (!blockCollisionCheck_)
        ODEPhysics::UpdateCollision();
}

LinkPtr OpptODEPhysics::CreateLink(ModelPtr _parent)
{
    if (_parent == NULL)
        gzthrow("Link must have a parent\n");

    std::map<std::string, dSpaceID>::iterator iter;
    iter = spaces_.find(_parent->GetName());

    if (iter == spaces_.end())
        spaces_[_parent->GetName()] =
            dSimpleSpaceCreate(GetSpaceId());

    OpptODELinkPtr link(new OpptODELink(_parent));

    link->SetSpaceId(spaces_[_parent->GetName()]);
    link->SetWorld(_parent->GetWorld());

    return link;

}

JointPtr OpptODEPhysics::CreateJoint(const std::string& _type, ModelPtr _parent)
{
    JointPtr joint;

    if (_type == "prismatic") {
        joint.reset(new OpptODEJoint<ODESliderJoint, dxJointSlider>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODESliderJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "screw") {
        joint.reset(new OpptODEJointExtended<ODEScrewJoint, dxJointScrew>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEScrewJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "revolute") {
        joint.reset(new OpptODEJointExtended<ODEHingeJoint, dxJointHinge>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEHingeJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "gearbox") {
        joint.reset(new OpptODEJointExtended2<ODEGearboxJoint, dxJointGearbox>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEGearboxJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "revolute2") {
        joint.reset(new OpptODEJoint<ODEHinge2Joint, dxJointHinge>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEHinge2Joint, dxJointHinge> *>(joint.get()));
    } else if (_type == "ball") {
        joint.reset(new OpptODEJoint<ODEBallJoint, dxJointBall>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEBallJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "universal") {
        joint.reset(new OpptODEJointExtended2<ODEUniversalJoint, dxJointUniversal>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEUniversalJoint, dxJointHinge> *>(joint.get()));
    } else if (_type == "fixed") {
        joint.reset(new OpptODEJoint<ODEFixedJoint, dxJointFixed>(GetWorldId(), _parent, &jointEntries_));
        jointEntries_[joint.get()] =
            static_cast<OpptJoint*>(static_cast<OpptODEJoint<ODEFixedJoint, dxJointHinge> *>(joint.get()));
    } else
        gzthrow("Unable to create joint of type[" << _type << "]");

    return joint;
}

void OpptODEPhysics::blockCollisionCheck(const bool& blockCollisionCheck)
{
    blockCollisionCheck_ = blockCollisionCheck;
}

bool OpptODEPhysics::collionCheckBlocked() const
{
    return blockCollisionCheck_;
}

void OpptODEPhysics::enableJoints(const bool& enable)
{
    for (auto & jointEntry : jointEntries_) {
        jointEntry.second->blockSetForce(!enable);
    }
}

CumulativeAnglesVec OpptODEPhysics::getCumulativeAngles() const
{
    CumulativeAnglesVec cumulativeAngles(jointNameMap_.size());
    size_t idx = 0;
    for (auto & jointEntry : jointNameMap_) {        
        cumulativeAngles[idx] =
            std::make_pair(jointEntry.first, jointNameMap_.at(jointEntry.first)->getCumulativeAngles());
        idx++;
    }

    return cumulativeAngles;
}

void OpptODEPhysics::setCumulativeAngles(const CumulativeAnglesVec& cumulativeAngles)
{
    for (auto & cumulaiveAngleEntry : cumulativeAngles) {
        jointNameMap_.at(cumulaiveAngleEntry.first)->setCumulativeAngles(cumulaiveAngleEntry.second);
    }
}

void OpptODEPhysics::makeJointNameMap()
{
    jointNameMap_ = std::unordered_map<std::string, OpptJoint*>();
    for (auto & jointEntry : jointEntries_) {        
        auto name = jointEntry.second->getName();
        jointNameMap_[name] = jointEntry.second;
    }
}

// Register OpptODEPhysics in Gazebo
GZ_REGISTER_PHYSICS_ENGINE("ode2", OpptODEPhysics)
