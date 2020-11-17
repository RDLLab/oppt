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
#ifndef _GOAL_PARSER_HPP_
#define _GOAL_PARSER_HPP_
#include "oppt/opptCore/core.hpp"
#include <sdf/parser.hh>

namespace oppt
{

struct DefaultTerminalPluginGoalParser
{
    VectorFloat toDoubleVec(VectorString& stringVec) {
        VectorFloat FloatTypeVec;
        for (size_t i = 0; i < stringVec.size(); i++) {
            FloatTypeVec.push_back(atof(stringVec[i].c_str()));
        }

        return FloatTypeVec;
    }

    Matrixdf processPoseElement(sdf::ElementPtr& poseElement) {
        if (poseElement) {
            if (poseElement->GetValue()) {
                std::string poseStr = poseElement->GetValue()->GetAsString();
                std::vector<std::string> elems;
                split(poseStr, ' ', elems);
                VectorFloat poseVec = toDoubleVec(elems);
                Matrix4f rotX = math::getRotationMatrixX(poseVec[3]);
                Matrix4f rotY = math::getRotationMatrixY(poseVec[4]);
                Matrix4f rotZ = math::getRotationMatrixZ(poseVec[5]);
                Matrix4f transMatrix = math::getTranslationMatrix(poseVec[0], poseVec[1], poseVec[2]);
                Matrix4f res = transMatrix * rotX * rotY * rotZ;
                return res;
            } else {
                oppt::WARNING("Pose of SDF element is malformed!");
            }
        }

        return Matrixdf::Identity(4, 4);
    }

    VectorFloat parseGoalAreaFromFile(const std::string& filename, const std::string &goalName = "GoalArea") {
        const std::string file = filename;
        sdf::SDFPtr sdfModel(new sdf::SDF());
        sdf::init(sdfModel);
        sdf::readFile(file, sdfModel);
        if (!sdfModel) {
            oppt::ERROR("Could not parse SDF file");
        }

        sdf::ElementPtr rootElement = sdfModel->Root();
        if (!rootElement) {
            ERROR("Couldn't parse SDF file");
        }

        sdf::ElementPtr worldElement = rootElement->GetElement("world");
        if (!worldElement) {
            ERROR("SDF file has no world element");
        }

        sdf::ElementPtr modelElement = worldElement->GetElement("model");
        if (!modelElement) {
            ERROR("SDF file has no model element");
        }

        while (modelElement) {
            std::string modelName = modelElement->Get<std::string>("name");
            if (modelName.find(goalName) != std::string::npos) {
                if (!modelElement->HasElement("pose")) {
                    ERROR("SDFParser: No pose defined for GoalArea element");
                }
                sdf::ElementPtr poseElement = modelElement->GetElement("pose");
                sdf::ElementPtr geometryElement;
                if (modelElement->GetElement("link")->HasElement("collision")) {
                    sdf::ElementPtr collisionElement =
                        modelElement->GetElement("link")->GetElement("collision");
                    if (!collisionElement->HasElement("geometry")) {
                        ERROR("SDFParser: goal area collision element has no geometry element");
                    }

                    geometryElement = collisionElement->GetElement("geometry");

                } else if (modelElement->GetElement("link")->HasElement("visual")) {
                    sdf::ElementPtr visualElement =
                        modelElement->GetElement("link")->GetElement("visual");
                    if (!visualElement->HasElement("geometry")) {
                        ERROR("SDFParser: goal area visual element has no geometry element");
                    }

                    geometryElement = visualElement->GetElement("geometry");
                }

                Matrix4f pose = processPoseElement(poseElement);
                if (geometryElement->HasElement("sphere")) {
                    sdf::ElementPtr sphereElement = geometryElement->GetElement("sphere");
                    if (!sphereElement->HasElement("radius")) {
                        ERROR("SDFParser: GoalArea sphere has no radius");
                    }

                    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
                    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
                    VectorFloat goalArea( {pose(0, 3), pose(1, 3), pose(2, 3), radius});
                    return goalArea;
                    //ERROR("SDFParser: GoalArea has no sphere element");
                }

                if (geometryElement->HasElement("box")) {
                    sdf::ElementPtr boxElement = geometryElement->GetElement("box");
                    if (!boxElement->HasElement("size")) {
                        ERROR("GoalArea box has no size element");
                    }

                    sdf::ElementPtr sizeElement = boxElement->GetElement("size");
                    std::string sizeStr = sizeElement->GetValue()->GetAsString();
                    VectorString sizeElems;
                    split(sizeStr, ' ', sizeElems);
                    if (sizeElems.size() != 3) {
                        ERROR("Size of box has incorrect number of values");                        
                    }

                    VectorFloat goalArea({pose(0, 3), 
                        pose(1, 3), 
                        pose(2, 3), 
                        atof(sizeElems[0].c_str()), 
                        atof(sizeElems[1].c_str()), 
                        atof(sizeElems[2].c_str())});
                    return goalArea;
                }
            }

            modelElement = modelElement->GetNextElement("model");
        }

        ERROR("No GoalArea defined in environment SDF");
        return VectorFloat();
    }
};




}
#endif
