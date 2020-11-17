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
#include "OpptODELink.hpp"
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include "OpptODEPhysics.hpp"
#include "oppt/opptCore/core.hpp"
#include "ode/objects.h"

using namespace gazebo;
using namespace physics;
using namespace oppt;

OpptODELink::OpptODELink(EntityPtr _parent):
    ODELink(_parent)
{

}

void OpptODELink::Init()
{
    ODELink::Init();

    // Replace the move callback with our own version
    if (GetODEId()) {
        dBodySetMovedCallback(GetODEId(), OpptMoveCallback);
        dBodySetDisabledCallback(GetODEId(), DisabledCallback);
    }
}

void OpptODELink::OpptMoveCallback(dBodyID _id)
{
    //ODELink::MoveCallback(_id);
    const dReal* p;
    const dReal* r;
    OpptODELink* self = static_cast<OpptODELink*>(dBodyGetData(_id));
    // self->poseMutex->lock();

    p = dBodyGetPosition(_id);
    r = dBodyGetQuaternion(_id);

#ifdef GZ_GT_7
    self->dirtyPose.Pos().Set(p[0], p[1], p[2]);
    self->dirtyPose.Rot().Set(r[0], r[1], r[2], r[3]);
#else
    self->dirtyPose.pos.Set(p[0], p[1], p[2]);
    self->dirtyPose.rot.Set(r[0], r[1], r[2], r[3]);
#endif


    // subtracting cog location from ode pose
    GZ_ASSERT(self->inertial != NULL, "Inertial pointer is NULL");
#ifdef GZ_GT_7
    GZVector3 cog = self->dirtyPose.Rot().RotateVector(
                        self->inertial->CoG());

    self->dirtyPose.Pos() -= cog;
#else
    GZVector3 cog = self->dirtyPose.rot.RotateVector(
                        self->inertial->GetCoG());

    self->dirtyPose.pos -= cog;
#endif

    // Tell the world that our pose has changed.
    //self->world->_AddDirty(self);

    // self->poseMutex->unlock();

    // get force and applied to this body
    if (_id) {
        const dReal* dforce = dBodyGetForce(_id);
        self->force.Set(dforce[0], dforce[1], dforce[2]);

        const dReal* dtorque = dBodyGetTorque(_id);
        self->torque.Set(dtorque[0], dtorque[1], dtorque[2]);
    }
}

void OpptODELink::DisabledCallback(dBodyID _id)
{

}

#ifdef GZ_GT_7
GZVector3 OpptODELink::WorldForce() const
{
    return this->force;
}

GZVector3 OpptODELink::WorldTorque() const
{
    return this->torque;
}
#else
GZVector3 OpptODELink::GetWorldForce() const
{
    return this->force;
}

GZVector3 OpptODELink::GetWorldTorque() const
{
    return this->torque;
}

#endif
