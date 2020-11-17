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
#ifndef __OPPT_MESH_HPP__
#define __OPPT_MESH_HPP__
#include "Geometry.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "oppt/opptCore/resources/resources.hpp"

namespace oppt
{

namespace geometric
{

/**
* Class that represents a mesh geometry
*/
class Mesh: public Geometry
{
public:
    using Geometry::Geometry;
    _NO_COPY_BUT_MOVE(Mesh)

    Mesh(const std::string& name,
         const std::string& meshFile,
         const Pose& worldPose,
         const VectorFloat& scale);

    virtual ~Mesh() = default;    
    
    virtual GeometryUniquePtr shallowCopy() const override;

    virtual std::string toSDFString() const override;

    /**
     * @brief Get the URI for this mesh
     */
    virtual std::string getMeshUri() const;

    /**
     * @brief Get the scale in xyz-direction
     */
    virtual VectorFloat getScale() const;

protected:
    virtual void createCollisionGeometry() override;

private:
    std::string meshFile_;

    VectorFloat scale_;

};

};

};

#endif
