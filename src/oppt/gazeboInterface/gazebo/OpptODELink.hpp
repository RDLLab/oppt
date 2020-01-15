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
#ifndef _OPPT_ODE_LINK_HPP_
#define _OPPT_ODE_LINK_HPP_
#include <gazebo/physics/ode/ODELink.hh>
#include <oppt/opptCore/typedefs.hpp>

using namespace oppt;

namespace gazebo
{
namespace physics
{
class OpptODELink: public ODELink
{
public:
	explicit OpptODELink(EntityPtr _parent);

	virtual void Init() override;

	static void OpptMoveCallback(dBodyID _id);

	static void DisabledCallback(dBodyID _id);

#ifdef GZ_GT_7
	virtual GZVector3 WorldForce() const override;

	virtual GZVector3 WorldTorque() const override;
#else
	virtual GZVector3 GetWorldForce() const override;

	virtual GZVector3 GetWorldTorque() const override;
#endif


private:
	GZVector3 force;

	GZVector3 torque;
};

typedef boost::shared_ptr<OpptODELink> OpptODELinkPtr;

}
}

#endif
