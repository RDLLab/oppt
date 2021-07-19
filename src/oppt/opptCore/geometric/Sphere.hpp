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
#ifndef __OPPT__SPHERE_HPP__
#define __OPPT__SPHERE_HPP__
#include "Geometry.hpp"

namespace oppt
{
namespace geometric
{
/**
* Class that represents a sphere geometry
*/
class Sphere: public Geometry
{
public:
    using Geometry::Geometry;
    _NO_COPY_BUT_MOVE(Sphere)
    
    Sphere(const std::string& name,
           const FloatType& radius,
           const Pose& worldPose);

    virtual ~Sphere() = default;    
    
    virtual GeometryUniquePtr shallowCopy() const override;

    /**
     * @brief Get the radius of the sphere
     */
    FloatType getRadius() const;

    virtual std::string toSDFString() const override;

protected:
    virtual void createCollisionGeometry() override;

private:
    FloatType radius_;
};


};

};

#endif
