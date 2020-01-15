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
/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef CYLINDER_BODY_HPP_
#define CYLINDER_BODY_HPP_
#include "Body.hpp"

namespace oppt
{

class CylinderBody: public BodyImpl
{
public:
    CylinderBody(const std::string &name,
                     const geometric::Pose &worldPose,
                     const FloatType& radius,
                     const FloatType& length);

    virtual BodyUniquePtr clone() const override;
};

}

#endif /* BODY_HPP_ */
