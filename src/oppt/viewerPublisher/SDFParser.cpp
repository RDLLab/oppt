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
#include "SDFParser.hpp"
#include "oppt/opptCore/resources/resources.hpp"

namespace oppt
{

SDFMarkersSharedPtr SDFParser::parseFromSDFString(std::string& sdfString, const std::string& robotName, const std::string &environmentFile) const
{
    SDFMarkersSharedPtr environmentMarkers(new SDFMarkers());

    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfString, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDFParser: sdf has no root element");
    }

    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    if (!worldElement) {
        ERROR("SDFParser: sdf has no world element");
    }

    std::string worldName = worldElement->Get<std::string>("name");
    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    if (worldName == "__default__") {
        modelElement = rootElement->GetElement("model");
    }

    if (!modelElement) {
        ERROR("SDFParser: sdf has no model element");
    }

    while (modelElement) {
        std::string modelName = modelElement->Get<std::string>("name");
        if (modelName != robotName) {
            sdf::ElementPtr modelPoseElement = modelElement->GetElement("pose");
            sdf::ElementPtr linkElement = modelElement->GetElement("link");
            while (linkElement) {
                sdf::ElementPtr linkPoseElement = linkElement->GetElement("pose");
                sdf::ElementPtr visualElement =
                    modelElement->GetElement("link")->GetElement("visual");

                if (visualElement) {
                    sdf::ElementPtr visualPoseElement = visualElement->GetElement("pose");
                    //geometric::Pose linkPose(processPoseElement(modelPoseElement).toGZPose() + processPoseElement(linkPoseElement).toGZPose());

                    sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
                    sdf::ElementPtr geometryTypeElement;
                    if (geometryElement) {
                        if (geometryElement->HasElement("mesh")) {
                            geometryTypeElement = geometryElement->GetElement("mesh");
                            processMeshElement(geometryTypeElement,
                                               modelPoseElement,
                                               visualElement,
                                               environmentMarkers,
                                               environmentFile);
                            environmentMarkers->markerNames.push_back(modelName);
                        } else if (geometryElement->HasElement("box")) {
                            geometryTypeElement = geometryElement->GetElement("box");
                            processBoxElement(geometryTypeElement,
                                              modelPoseElement,
                                              visualElement,
                                              environmentMarkers);
                            environmentMarkers->markerNames.push_back(modelName);
                        } else if (geometryElement->HasElement("sphere")) {
                            geometryTypeElement = geometryElement->GetElement("sphere");
                            processSphereElement(geometryTypeElement,
                                                 modelPoseElement,
                                                 visualElement,
                                                 environmentMarkers);
                            environmentMarkers->markerNames.push_back(modelName);
                        } else if (geometryElement->HasElement("cylinder")) {
                            geometryTypeElement = geometryElement->GetElement("cylinder");
                            processCylinderElement(geometryTypeElement,
                                                   modelPoseElement,
                                                   visualElement,
                                                   environmentMarkers);
                        }
                    }
                }

                linkElement = linkElement->GetNextElement("link");
            }
        }

        modelElement = modelElement->GetNextElement("model");
    }

    return environmentMarkers;

}

SDFMarkersSharedPtr SDFParser::parseFromFile(const std::string& environmentFile, const std::string& robotName) const
{
    const std::string file = environmentFile;
    std::ifstream ifs(file);
    std::string sdfString((std::istreambuf_iterator<char>(ifs)),
                          (std::istreambuf_iterator<char>()));
    return parseFromSDFString(sdfString, robotName, environmentFile);
}

geometry_msgs::Pose SDFParser::processPoseElement(sdf::ElementPtr& poseElement) const
{
    std::string poseStr;
    geometry_msgs::Pose pose;
    if (poseElement) {
        if (poseElement->GetValue()) {
            poseStr = poseElement->GetValue()->GetAsString();
            std::vector<std::string> elems;
            split(poseStr, ' ', elems);
            pose.position.x = atof(elems[0].c_str());
            pose.position.y = atof(elems[1].c_str());
            pose.position.z = atof(elems[2].c_str());
            Quaternionf quat = math::eulerAnglesToQuaternion(atof(elems[3].c_str()),
                               atof(elems[4].c_str()),
                               atof(elems[5].c_str()));
            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();
        } else {
            oppt::WARNING("Pose of SDF element is malformed!");
        }
    }

    return pose;
}

bool SDFParser::processMaterialElement(std::shared_ptr<visualization_msgs::Marker>& marker,
                                       sdf::ElementPtr& visualElement) const
{
    marker->color.a = 0.1;
    marker->color.r = 0.1;
    marker->color.g = 0.1;
    marker->color.b = 0.1;
    sdf::ElementPtr materialElement = visualElement->GetElement("material");
    if (materialElement) {
        if (materialElement->HasElement("ambient")) {
            sdf::ElementPtr ambientElement = materialElement->GetElement("ambient");
            std::string colorString = ambientElement->GetValue()->GetAsString().c_str();
            VectorString colorVec;
            split(colorString, ' ', colorVec);
            marker->color.r = atof(colorVec[0].c_str());
            marker->color.g = atof(colorVec[1].c_str());
            marker->color.b = atof(colorVec[2].c_str());
            marker->color.a = atof(colorVec[3].c_str());
        }
    }

    return true;
}

SDFMarkersSharedPtr SDFParser::textToMarkers(const std::string& text) const
{
    SDFMarkersSharedPtr textMarkers(new SDFMarkers());
    MarkerSharedPtr marker = std::make_shared<visualization_msgs::Marker>();
    ////////////////////////////////////////
    geometry_msgs::Pose markerPose;
    markerPose.position.x = 0;
    markerPose.position.y = 0;
    markerPose.position.z = 0;

    markerPose.orientation.x = 0;
    markerPose.orientation.y = 0;
    markerPose.orientation.z = 0;
    markerPose.orientation.w = 1;

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker->action = visualization_msgs::Marker::ADD;
    marker->text = text;

    marker->scale.x = 1;
    marker->scale.y = 1;
    marker->scale.z = 1;

    marker->color.r = 1;
    marker->color.g = 1;
    marker->color.b = 1;
    marker->color.a = 1;
    ////////////////////////////////////////
    textMarkers->markerNames.push_back("textMarker");
    textMarkers->markers.push_back(marker);
    return textMarkers;
}

MarkerSharedPtr SDFParser::sphereToMarker(const geometric::Geometry *sphereGeometry) const
{
    const geometric::Sphere* sphere = static_cast<const geometric::Sphere*>(sphereGeometry);
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    FloatType radius = sphere->getRadius();
    geometric::Pose pose = sphere->getWorldPose();
    VectorFloat color = sphere->getColor();

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose.position.x();
    markerPose.position.y = pose.position.y();
    markerPose.position.z = pose.position.z();

    markerPose.orientation.x = pose.orientation.x();
    markerPose.orientation.y = pose.orientation.y();
    markerPose.orientation.z = pose.orientation.z();
    markerPose.orientation.w = pose.orientation.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::SPHERE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2.0 * radius;
    marker->scale.y = 2.0 * radius;
    marker->scale.z = 2.0 * radius;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

MarkerSharedPtr SDFParser::cylinderToMarker(const geometric::Geometry *cylinderGeometry) const
{
    const geometric::Cylinder* cylinder = static_cast<const geometric::Cylinder*>(cylinderGeometry);
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    FloatType radius = cylinder->getRadius();
    FloatType length = cylinder->getLength();
    geometric::Pose pose = cylinder->getWorldPose();
    VectorFloat color = cylinder->getColor();
    if (color.size() != 4)
        ERROR("Color vec has wrong size");

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose.position.x();
    markerPose.position.y = pose.position.y();
    markerPose.position.z = pose.position.z();

    markerPose.orientation.x = pose.orientation.x();
    markerPose.orientation.y = pose.orientation.y();
    markerPose.orientation.z = pose.orientation.z();
    markerPose.orientation.w = pose.orientation.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2.0 * radius;
    marker->scale.y = 2.0 * radius;
    marker->scale.z = length;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

MarkerSharedPtr SDFParser::boxToMarker(const geometric::Geometry *boxGeometry) const
{
    const geometric::Box* box = static_cast<const geometric::Box*>(boxGeometry);
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    geometric::Pose pose = box->getWorldPose();
    VectorFloat dimensions = box->getDimensions();
    VectorFloat color = box->getColor();

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose.position.x();
    markerPose.position.y = pose.position.y();
    markerPose.position.z = pose.position.z();

    markerPose.orientation.x = pose.orientation.x();
    markerPose.orientation.y = pose.orientation.y();
    markerPose.orientation.z = pose.orientation.z();
    markerPose.orientation.w = pose.orientation.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = dimensions[0];
    marker->scale.y = dimensions[1];
    marker->scale.z = dimensions[2];
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

MarkerSharedPtr SDFParser::meshToMarker(const geometric::Geometry *meshGeometry) const
{
    const geometric::Mesh* mesh = static_cast<const geometric::Mesh*>(meshGeometry);
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    geometric::Pose pose = mesh->getWorldPose();
    VectorFloat color = mesh->getColor();
    std::string meshUri = mesh->getMeshUri();
    meshUri = oppt::resources::FindFile(meshUri);
    VectorFloat scale = mesh->getScale();

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose.position.x();
    markerPose.position.y = pose.position.y();
    markerPose.position.z = pose.position.z();

    markerPose.orientation.x = pose.orientation.x();
    markerPose.orientation.y = pose.orientation.y();
    markerPose.orientation.z = pose.orientation.z();
    markerPose.orientation.w = pose.orientation.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = scale[0];
    marker->scale.y = scale[1];
    marker->scale.z = scale[2];
    marker->mesh_resource = "file://" + meshUri;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    marker->mesh_use_embedded_materials = true;
    return marker;
}

bool SDFParser::processSphereElement(sdf::ElementPtr& sphereElement,
                                     sdf::ElementPtr& poseElement,
                                     sdf::ElementPtr& visualElement,
                                     SDFMarkersSharedPtr& environmentMarkers) const
{
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement) {
        oppt::WARNING("Sphere has no radius element");
        return false;
    }

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::SPHERE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2 * radius;
    marker->scale.y = 2 * radius;
    marker->scale.z = 2 * radius;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;

}

bool SDFParser::processCylinderElement(sdf::ElementPtr& cylinderElement,
                                       sdf::ElementPtr& poseElement,
                                       sdf::ElementPtr& visualElement,
                                       SDFMarkersSharedPtr& environmentMarkers) const
{
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement)
        oppt::WARNING("Cylinder has no radius element");
    if (!lengthElement)
        oppt::WARNING("Cylinder has no length element");
    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2 * radius;
    marker->scale.y = 2 * radius;
    marker->scale.z = length;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

MarkerSharedPtr SDFParser::geometryToMarker(const geometric::Geometry *geometry,
        const geometric::Pose& pose,
        const FloatType& opacity) const
{
    MarkerSharedPtr marker;
    std::string name;
    bool validGeometry = true;
    switch (geometry->getType()) {
    case geometric::BOX:
        marker = boxToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::SPHERE:
        marker = sphereToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::CYLINDER:
        marker = cylinderToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::MESH:
        marker = meshToMarker(geometry);
        name = geometry->getName();
        break;
    default:
        WARNING("SDFParser: geometriesToMarkers: geometry type not recognized: " + std::to_string(geometry->getType()));
        validGeometry = false;
        break;
    }

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose.position.x();
    markerPose.position.y = pose.position.y();
    markerPose.position.z = pose.position.z();

    markerPose.orientation.x = pose.orientation.x();
    markerPose.orientation.y = pose.orientation.y();
    markerPose.orientation.z = pose.orientation.z();
    markerPose.orientation.w = pose.orientation.w();
    marker->pose = markerPose;

    VectorFloat geomColor = geometry->getColor();
    marker->color.r = geomColor[0];
    marker->color.g = geomColor[1];
    marker->color.b = geomColor[2];
    marker->color.a = opacity;

    return marker;
}

SDFMarkersSharedPtr SDFParser::geometriesToMarkers(VectorGeometryPtr& geometries) const
{
    SDFMarkersSharedPtr environmentMarkers(new SDFMarkers());
    for (auto & geometry : geometries) {
        MarkerSharedPtr marker;
        std::string name;
        bool validGeometry = true;
        switch (geometry->getType()) {
        case geometric::BOX:
            marker = boxToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::SPHERE:
            marker = sphereToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::CYLINDER:
            marker = cylinderToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::MESH:
            marker = meshToMarker(geometry);
            name = geometry->getName();
            break;
        default:
            WARNING("SDFParser: geometriesToMarkers: geometry type not recognized: " + std::to_string(geometry->getType()));
            validGeometry = false;
            break;
        }

        if (validGeometry) {
            std::vector<MarkerSharedPtr> frameMarkers;// = geometryToFrameMarkers(geometry);
            environmentMarkers->markerNames.push_back(name);
            environmentMarkers->markers.push_back(marker);
            for (size_t i = 0; i != frameMarkers.size(); ++i) {
                environmentMarkers->markerNames.push_back(name + "_frame_" + std::to_string(i));
                environmentMarkers->markers.push_back(frameMarkers[i]);
            }
        }
    }


    return environmentMarkers;
}

bool SDFParser::processBoxElement(sdf::ElementPtr& boxElement,
                                  sdf::ElementPtr& poseElement,
                                  sdf::ElementPtr& visualElement,
                                  SDFMarkersSharedPtr& environmentMarkers) const
{
    sdf::ElementPtr sizeElement = boxElement->GetElement("size");
    if (!sizeElement) {
        oppt::WARNING("Box has no size element");
        return false;
    }

    std::string sizeStr = sizeElement->GetValue()->GetAsString();
    VectorString sizeElems;
    split(sizeStr, ' ', sizeElems);
    if (sizeElems.size() != 3) {
        oppt::WARNING("Size of box has incorrect number of values");
        return false;
    }

    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 1.0;
    marker->scale.y = 1.0;
    marker->scale.z = 1.0;
    if (sizeElems.size() > 0) {
        marker->scale.x = atof(sizeElems[0].c_str());
        marker->scale.y = atof(sizeElems[1].c_str());
        marker->scale.z = atof(sizeElems[2].c_str());
    }

    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

bool SDFParser::processMeshElement(sdf::ElementPtr& meshElement,
                                   sdf::ElementPtr& poseElement,
                                   sdf::ElementPtr& visualElement,
                                   SDFMarkersSharedPtr& environmentMarkers,
                                   const std::string& environmentFile) const
{
    if (meshElement->HasAttribute("scale")) {

    }

    std::string meshUri = meshElement->GetElement("uri")->GetValue()->GetAsString();
    meshUri = oppt::resources::FindFile(meshUri);
    std::string scaleStr = meshElement->GetElement("scale")->GetValue()->GetAsString();
    std::vector<std::string> scaleVecElemes;
    split(scaleStr, ' ', scaleVecElemes);
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;

    // Process scale here

    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 1.0;
    marker->scale.y = 1.0;
    marker->scale.z = 1.0;
    if (scaleVecElemes.size() > 0) {
        marker->scale.x = atof(scaleVecElemes[0].c_str());
        marker->scale.y = atof(scaleVecElemes[1].c_str());
        marker->scale.z = atof(scaleVecElemes[2].c_str());
    }

    marker->mesh_resource = "file://" + meshUri;
    marker->mesh_use_embedded_materials = true;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

int SDFParser::getMarkerId(SDFMarkersSharedPtr& environmentMarkers) const
{
    int markerId = 0;
    if (environmentMarkers->markers.size() > 0) {
        markerId = environmentMarkers->markers[environmentMarkers->markers.size() - 1]->id + 1;
    }

    return markerId;
}

}
