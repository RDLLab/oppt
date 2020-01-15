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

VectorGeometryPtr RobotImplSDFParser::parseCollisionGeometries(const std::string& robotName, const std::string &environmentFile) const
{
    VectorGeometryPtr robotGeometries;
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
                            robotGeometries.push_back(processBoxGeometry(geometryElement, collisionName, nullptr));
                        } else if (geometryElement->HasElement("mesh")) {
                            robotGeometries.push_back(processMeshGeometry(geometryElement, collisionName, nullptr));
                        } else if (geometryElement->HasElement("cylinder")) {
                            robotGeometries.push_back(processCylinderGeometry(geometryElement, collisionName, nullptr));
                        } else if (geometryElement->HasElement("sphere")) {
                            robotGeometries.push_back(processSphereGeometry(geometryElement, collisionName, nullptr));
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

    return robotGeometries;
}

VectorGeometryPtr RobotImplSDFParser::parseVisualGeometries(const std::string& robotName, const std::string &environmentFile) const
{
    VectorGeometryPtr robotGeometries;
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
                            robotGeometries.push_back(processBoxGeometry(geometryElement,
                                                      visualName,
                                                      visualElement));
                        } else if (geometryElement->HasElement("mesh")) {
                            robotGeometries.push_back(processMeshGeometry(geometryElement,
                                                      visualName,
                                                      visualElement));
                        } else if (geometryElement->HasElement("sphere")) {
                            auto sphereGeometry = processSphereGeometry(geometryElement,
                                                  visualName,
                                                  visualElement);
                            if (sphereGeometry)
                                robotGeometries.push_back(sphereGeometry);
                        } else if (geometryElement->HasElement("cylinder")) {
                            robotGeometries.push_back(processCylinderGeometry(geometryElement,
                                                      visualName,
                                                      visualElement));
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

    return robotGeometries;
}

void RobotImplSDFParser::processMaterialElement(const sdf::ElementPtr& visualElement, GeometrySharedPtr& geometryElem) const
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

GeometrySharedPtr RobotImplSDFParser::processBoxGeometry(sdf::ElementPtr boxGeometryElem,
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
    GeometrySharedPtr boxGeometry = std::make_shared<oppt::geometric::Box>(visualName, sizeVec, geometric::Pose());
    processMaterialElement(visualElement, boxGeometry);
    return boxGeometry;
}

GeometrySharedPtr RobotImplSDFParser::processSphereGeometry(sdf::ElementPtr sphereGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr sphereElem = sphereGeometryElem->GetElement("sphere");
    sdf::ElementPtr radiusElement = sphereElem->GetElement("radius");
    if (radiusElement) {
        std::string radiusStr = radiusElement->GetValue()->GetAsString();
        FloatType radius = atof(radiusStr.c_str());
        geometric::Pose pose;
        GeometrySharedPtr sphereGeometry = std::make_shared<oppt::geometric::Sphere>(visualName, radius, geometric::Pose());
        processMaterialElement(visualElement, sphereGeometry);
        return sphereGeometry;
    } else {
        WARNING("Visual element '" + visualName + "' is a sphere, but has no radius. Ignoring.");
        return nullptr;
    }

}

GeometrySharedPtr RobotImplSDFParser::processCylinderGeometry(sdf::ElementPtr cylinderGeometryElem,
        const std::string& visualName,
        const sdf::ElementPtr& visualElement) const
{
    sdf::ElementPtr cylinderElem = cylinderGeometryElem->GetElement("cylinder");
    sdf::ElementPtr radiusElement = cylinderElem->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElem->GetElement("length");
    if (radiusElement && lengthElement) {
        std::string radiusStr = radiusElement->GetValue()->GetAsString();
        std::string lengthStr = lengthElement->GetValue()->GetAsString();
        FloatType radius = atof(radiusStr.c_str());
        FloatType length = atof(lengthStr.c_str());
        geometric::Pose pose;
        GeometrySharedPtr cylinderGeometry = std::make_shared<oppt::geometric::Cylinder>(visualName, radius, length, geometric::Pose());
        processMaterialElement(visualElement, cylinderGeometry);
        return cylinderGeometry;
    } else {
        WARNING("Visual element '" + visualName + "' is a cylinder, but has radius or length missing. Ignoring.");
        return nullptr;
    }

}

GeometrySharedPtr RobotImplSDFParser::processMeshGeometry(sdf::ElementPtr meshGeometryElem,
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
    GeometrySharedPtr meshGeometry = std::make_shared<oppt::geometric::Mesh>(visualName, meshUri, geometric::Pose(), scale);
    processMaterialElement(visualElement, meshGeometry);
    return meshGeometry;
}

}
