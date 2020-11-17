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
#ifndef __OPPT_CYLINDER_HPP__
#define __OPPT_CYLINDER_HPP__
#include "Geometry.hpp"

namespace oppt
{
namespace geometric
{

/**
 * Class that represents a cylinder geometry
 */
class Cylinder: public Geometry
{
public:
    using Geometry::Geometry;
    _NO_COPY_BUT_MOVE(Cylinder)

    Cylinder(const std::string& name,
             const FloatType& radius,
             const FloatType& length,
             const Pose& worldPose);

    virtual ~Cylinder() = default;

    virtual GeometryUniquePtr shallowCopy() const override;

    /**
     * @brief Get the radius of the cylinder
     */
    FloatType getRadius() const;

    /**
     * @brief Get the length of the cylinder
     */
    FloatType getLength() const;

    virtual std::string toSDFString() const override;

protected:
    virtual void createCollisionGeometry() override;

private:
    FloatType radius_;

    FloatType length_;
};


};

};

#endif
