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
#ifndef MESH_BODY_HPP_
#define MESH_BODY_HPP_
#include "Body.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"

namespace oppt
{

class MeshBody: public BodyImpl
{
public:
    MeshBody(const std::string &name,
                 const VectorString& meshFiles,
                 const geometric::Pose& worldPose,
                 const VectorFloat& scale,
                 const bool &loadFromFile = true);

    virtual BodyUniquePtr clone() const override;    

    void removeVertices(std::vector<VectorFloat>& vertices);

private:
    fcl::Transform3f identity_transform_;

    VectorString meshFiles_;

    VectorFloat scale_;
};

}

#endif /* BODY_HPP_ */
