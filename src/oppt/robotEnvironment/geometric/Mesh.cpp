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
#include "oppt/opptCore/geometric/Mesh.hpp"
#include "oppt/opptCore/MathUtils.hpp"
#include <gazebo/common/MeshManager.hh>

namespace oppt
{
namespace geometric
{
Mesh::Mesh(const std::string& name,
           const std::string& meshFile,
           const Pose& worldPose,
           const VectorFloat& scale):
    Geometry(MESH, name, worldPose),
    meshFile_(meshFile),
    scale_(scale) {
    
} 

GeometryUniquePtr Mesh::shallowCopy() const {
    std::string name = name_;
    Pose worldPose(worldPose_);    
    std::string meshFile = meshFile_;
    VectorFloat scale = scale_;
    GeometryUniquePtr copiedGeometry(new Mesh(name, meshFile, worldPose, scale));
    copiedGeometry->setColor(getColor());
    return std::move(copiedGeometry);   
}

std::string Mesh::toSDFString() const {
    std::string serString = "";
    serString += "<geometry>";
    serString += "<mesh>";
    serString += "<scale>";
    serString += std::to_string(scale_[0]) + " ";
    serString += std::to_string(scale_[1]) + " ";
    serString += std::to_string(scale_[2]);
    serString += "</scale>";
    serString += "<uri>";
    serString += meshFile_;
    serString += "</uri>";
    serString += "</mesh>";
    serString += "</geometry>";
    return serString;
}

std::string Mesh::getMeshUri() const {
    return meshFile_;
}

VectorFloat Mesh::getScale() const {
    return scale_;
}

void Mesh::createCollisionGeometry() {
    std::string meshFile = oppt::resources::FindFile(meshFile_);    
    Assimp::Importer importer;
    unsigned int flags = aiProcess_PreTransformVertices | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices;    
    const aiScene* scene = importer.ReadFile(meshFile, flags);
    if (!scene)
        ERROR("Couldn't load mesh '" + meshFile + "'");
    collisionGeometry_ = FCLCollisionGeometrySharedPtr(new fcl::BVHModel<fcl::OBBRSS>());
    Matrixdf preRotation = Matrixdf::Identity(4, 4);

    // Get file extension
    VectorString fileStrElems;
    split(meshFile, ".", fileStrElems);
    if (fileStrElems[fileStrElems.size() - 1] != "stl") {
        std::ifstream ifs(meshFile);
        std::string content((std::istreambuf_iterator<char>(ifs)),
                            (std::istreambuf_iterator<char>()));
        if (content.find("<up_axis>Z_UP") != std::string::npos)
            preRotation = math::getRotationMatrixX(90.0 * M_PI / 180.0);

    }

    Matrixdf scaleMatrix = Matrixdf::Identity(4, 4);
    scaleMatrix(0, 0) = scale_[0];
    scaleMatrix(1, 1) = scale_[1];
    scaleMatrix(2, 2) = scale_[1];
    static_cast<fcl::BVHModel<fcl::OBBRSS> *>(collisionGeometry_.get())->beginModel();    
    for (std::size_t i = 0; i < scene->mNumMeshes; i++) {        
        for (std::size_t j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {            
            std::vector<fcl::Vec3f> vertices(3);
            for (std::size_t k = 0; k < 3; k++) {                
                const aiVector3D& vertex
                    = scene->mMeshes[i]->mVertices[
                          scene->mMeshes[i]->mFaces[j].mIndices[k]];
                if (!(&vertex))
                    ERROR("Vertex is null");                
                Vectordf v(4);
                v << vertex.x, vertex.y, vertex.z, 1.0;
                v = scaleMatrix * v;
                v = preRotation * v;
                vertices[k] = fcl::Vec3f(v[0],
                                         v[1],
                                         v[2]);
            }

            static_cast<fcl::BVHModel<fcl::OBBRSS> *>(collisionGeometry_.get())->addTriangle(vertices[0], 
            vertices[1], 
            vertices[2]);
        }
    }

    static_cast<fcl::BVHModel<fcl::OBBRSS> *>(collisionGeometry_.get())->endModel();
}

}
}


