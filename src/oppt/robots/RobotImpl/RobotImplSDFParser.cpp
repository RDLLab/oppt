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
#include "oppt/robotHeaders/RobotImpl/RobotImplSDFParser.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"
#include "oppt/opptCore/geometric/Box.hpp"
#include "oppt/opptCore/geometric/Sphere.hpp"
#include "oppt/opptCore/geometric/Cylinder.hpp"

namespace oppt
{
RobotImplSDFParser::RobotImplSDFParser()
{

}

VectorGeometryUniquePtr RobotImplSDFParser::parseCollisionGeometries(const std::string& robotName, const std::string &environmentFile) const
{
    VectorGeometryUniquePtr robotGeometries;
    if (environmentFile.empty())
        return std::move(robotGeometries);
    const std::string file = environmentFile;
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(file, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDF file seems invalid");
    }

    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    if (!worldElement)
        ERROR("SDF File has no world");

    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    if (!modelElement) {
        ERROR("SDF file has no model element");
    }

    while (modelElement) {
        std::string modelName = modelElement->Get<std::string>("name");
        if (modelName == robotName) {
            sdf::ElementPtr linkElement = modelElement->GetElement("link");
            while (linkElement) {
                std::string linkName = linkElement->Get<std::string>("name");
                sdf::ElementPtr collisionElement = linkElement->GetElement("collision");
                while (collisionElement) {
                    std::string collisionName = modelName + "::" + linkName + "::" + collisionElement->Get<std::string>("name");
                    sdf::ElementPtr geometryElement = collisionElement->GetElement("geometry");
                    while (geometryElement) {
                        if (geometryElement->HasElement("box")) {
                            robotGeometries.push_back(std::move(processBoxGeometry(geometryElement, collisionName, nullptr)));
                        } else if (geometryElement->HasElement("mesh")) {
                            robotGeometries.push_back(std::move(processMeshGeometry(geometryElement, collisionName, nullptr)));
                        } else if (geometryElement->HasElement("cylinder")) {
                            robotGeometries.push_back(std::move(processCylinderGeometry(geometryElement, collisionName, nullptr)));
                        } else if (geometryElement->HasElement("sphere")) {
                            robotGeometries.push_back(std::move(processSphereGeometry(geometryElement, collisionName, nullptr)));
                        }

                        geometryElement = geometryElement->GetNextElement("geometry");
                    }

                    collisionElement = collisionElement->GetNextElement("collision");
                }

                linkElement = linkElement->GetNextElement("link");
            }
        }

        modelElement = modelElement->GetNextElement("model");
    }

    if (robotGeometries.empty())
        ERROR("Your robot doesn't have any physical geometries");

    return std::move(robotGeometries);
}

VectorGeometryUniquePtr RobotImplSDFParser::parseVisualGeometries(const std::string& robotName, const std::string &environmentFile) const
{
    VectorGeometryUniquePtr robotGeometries;
    if (environmentFile.empty())
        return std::move(robotGeometries);
    const std::string file = environmentFile;
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(file, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDF file seems invalid");
    }

    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    if (!worldElement)
        ERROR("SDF File has no world element");

    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    if (!modelElement) {
        ERROR("SDF file has no model element");
    }

    while (modelElement) {
        std::string modelName = modelElement->Get<std::string>("name");
        //if (modelName == robotName) {
        sdf::ElementPtr linkElement = modelElement->GetElement("link");
        while (linkElement) {
            std::string linkName = linkElement->Get<std::string>("name");
            sdf::ElementPtr visualElement = linkElement->GetElement("visual");
            while (visualElement) {
                std::string visualName = modelName + "::" + linkName + "::" + visualElement->Get<std::string>("name");
                sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
                while (geometryElement) {
                    if (geometryElement->HasElement("box")) {
                        robotGeometries.push_back(std::move(processBoxGeometry(geometryElement,
                                                            visualName,
                                                            visualElement)));
                    } else if (geometryElement->HasElement("mesh")) {
                        robotGeometries.push_back(std::move(processMeshGeometry(geometryElement,
                                                            visualName,
                                                            visualElement)));
                    } else if (geometryElement->HasElement("sphere")) {
                        robotGeometries.push_back(processSphereGeometry(geometryElement,
                                                  visualName,
                                                  visualElement));
                    } else if (geometryElement->HasElement("cylinder")) {
                        robotGeometries.push_back(std::move(processCylinderGeometry(geometryElement,
                                                            visualName,
                                                            visualElement)));
                    }

                    geometryElement = geometryElement->GetNextElement("geometry");
                }

                visualElement = visualElement->GetNextElement("visual");
            }

            linkElement = linkElement->GetNextElement("link");
        }
        //}

        modelElement = modelElement->GetNextElement("model");
    }

    if (robotGeometries.empty())
        WARNING("Your robot doesn't have any visual geometries");

    return std::move(robotGeometries);
}

void RobotImplSDFParser::processMaterialElement(const sdf::ElementPtr& visualElement, geometric::Geometry *geometryElem) const
{
    if (visualElement) {
        sdf::ElementPtr materialElement = visualElement->GetElement("material");
        if (materialElement) {
            sdf::ElementPtr ambientElement = materialElement->GetElement("ambient");
            if (ambientElement) {
                std::string colorStr = ambientElement->GetValue()->GetAsString();
                VectorString colorStrVec;
                split(colorStr, ' ', colorStrVec);
                if (colorStrVec.size() == 4) {
                    VectorFloat colorVec(4);
                    for (size_t i = 0; i != colorStrVec.size(); ++i) {
                        colorVec[i] = atof(colorStrVec[i].c_str());
                    }

                    if (colorVec[0] == 0.0 && colorVec[1] == 0.0 && colorVec[2] == 0.0) {
                        colorVec = VectorFloat({0.5, 0.5, 0.5, colorVec[3]});
                    }
                    geometryElem->setColor(colorVec);
                }
            }
        }
    }
}

GeometryUniquePtr RobotImplSDFParser::processBoxGeometry(sdf::ElementPtr boxGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr boxElem = boxGeometryElem->GetElement("box");
    sdf::ElementPtr sizeElement = boxElem->GetElement("size");
    std::string sizeStr = sizeElement->GetValue()->GetAsString();
    VectorString sizeStrVec;
    split(sizeStr, ' ', sizeStrVec);
    VectorFloat sizeVec(sizeStrVec.size());
    for (size_t i = 0; i < sizeStrVec.size(); i++) {
        sizeVec[i] = atof(sizeStrVec[i].c_str());
    }

    geometric::Pose pose;
    GeometryUniquePtr boxGeometry(new geometric::Box(visualName, sizeVec, geometric::Pose()));
    processMaterialElement(visualElement, boxGeometry.get());    
    return std::move(boxGeometry);
}

GeometryUniquePtr RobotImplSDFParser::processSphereGeometry(sdf::ElementPtr sphereGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr sphereElem = sphereGeometryElem->GetElement("sphere");
    sdf::ElementPtr radiusElement = sphereElem->GetElement("radius");
    std::string radiusStr = radiusElement->GetValue()->GetAsString();
    FloatType radius = atof(radiusStr.c_str());
    GeometryUniquePtr sphereGeometry(new geometric::Sphere(visualName, radius, geometric::Pose()));
    processMaterialElement(visualElement, sphereGeometry.get());    
    return std::move(sphereGeometry);
}

GeometryUniquePtr RobotImplSDFParser::processCylinderGeometry(sdf::ElementPtr cylinderGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr cylinderElem = cylinderGeometryElem->GetElement("cylinder");
    sdf::ElementPtr radiusElement = cylinderElem->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElem->GetElement("length");
    std::string radiusStr = radiusElement->GetValue()->GetAsString();
    std::string lengthStr = lengthElement->GetValue()->GetAsString();
    FloatType radius = atof(radiusStr.c_str());
    FloatType length = atof(lengthStr.c_str());
    GeometryUniquePtr cylinderGeometry(new geometric::Cylinder(visualName, radius, length, geometric::Pose()));
    processMaterialElement(visualElement, cylinderGeometry.get());    
    return std::move(cylinderGeometry);
}

GeometryUniquePtr RobotImplSDFParser::processMeshGeometry(sdf::ElementPtr meshGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr meshElement = meshGeometryElem->GetElement("mesh");
    std::string meshUri = meshElement->GetElement("uri")->GetValue()->GetAsString();
    std::string scaleStr = meshElement->GetElement("scale")->GetValue()->GetAsString();
    VectorString scaleStringElems;
    split(scaleStr, ' ', scaleStringElems);
    VectorFloat scale(scaleStringElems.size());
    for (size_t i = 0; i < scaleStringElems.size(); i++) {
        scale[i] = atof(scaleStringElems[i].c_str());
    }

    geometric::Pose pose;

    if (!oppt::resources::FileExists(meshUri))
        ERROR("File: '" + meshUri + "' doesn't exist");

    std::string meshFile = oppt::resources::FindFile(meshUri);
    GeometryUniquePtr meshGeometry(new geometric::Mesh(visualName, meshUri, geometric::Pose(), scale));    
    processMaterialElement(visualElement, meshGeometry.get());    
    return std::move(meshGeometry);
}

}
