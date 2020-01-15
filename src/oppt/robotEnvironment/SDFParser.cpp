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
#include "include/SDFParser.hpp"
#include "oppt/opptCore/resources/resources.hpp"
#include "include/BoxBody.hpp"
#include "include/MeshBody.hpp"
#include "include/SphereBody.hpp"
#include "include/CylinderBody.hpp"
#include <oppt/opptCore/geometric/Box.hpp>
#include <oppt/opptCore/geometric/Sphere.hpp>
#include <oppt/opptCore/geometric/Cylinder.hpp>
#include <oppt/opptCore/geometric/Mesh.hpp>

namespace oppt
{

SDFEnvironmentParser::SDFEnvironmentParser()
{

}

VectorBodyUniquePtr SDFEnvironmentParser::parseBodiesFromSDFString(const std::string& sdfString) const
{
    std::string sdfStr(sdfString);
    if (sdfStr.find("<sdf") == std::string::npos) {
        sdfStr = "<sdf version='1.6'>" + sdfStr + "</sdf>";
    }

    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfStr, sdfModel);
    if (!sdfModel) {
        ERROR("SDFParser: Could not parse SDF string");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDFParser: Invalid SDF string");
    }

    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    if (!worldElement)
        ERROR("SDF doesn't contain a world");

    sdf::ElementPtr modelElement = nullptr;
    if (worldElement->Get<std::string>("name") == "__default__")
        modelElement = rootElement->GetElement("model");
    else
        modelElement = worldElement->GetElement("model");
    if (!modelElement) {
        ERROR("SDFParser: Invalid SDF string");
    }

    std::string modelName = modelElement->Get<std::string>("name");
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
    VectorBodyUniquePtr bodies;
    sdf::ElementPtr linkElement = modelElement->GetElement("link");
    while (linkElement) {
        BodyUniquePtr body = std::move(processLinkElement(modelName, poseElement, linkElement));
        if (body)
            bodies.push_back(std::move(body));
        linkElement = linkElement->GetNextElement("link");
    }

    return bodies;
}

BodyUniquePtr SDFEnvironmentParser::processLinkElement(const std::string& modelName,
        const sdf::ElementPtr& modelPoseElement,
        const sdf::ElementPtr& linkElement) const
{
    BodyUniquePtr body = nullptr;
    sdf::ElementPtr collisionElement =
        linkElement->GetElement("collision");
    sdf::ElementPtr visualElement =
        linkElement->GetElement("visual");
    sdf::ElementPtr linkPoseElement = linkElement->GetElement("pose");

    geometric::Pose linkWorldPose(processPoseElement(linkPoseElement).toGZPose() + processPoseElement(modelPoseElement).toGZPose());
    auto name = linkElement->Get<std::string>("name");
    bool enabled = true;
    sdf::ElementPtr geometryElement;
    std::string collisionName = collisionElement->Get<std::string>("name");
    if (collisionName == "__default__") {
        enabled = false;
        std::string visualName = visualElement->Get<std::string>("name");
        if (visualName == "__default__") {
            std::string msg = "SDFParser: Body ";
            msg += name;
            msg += " has no collision or visual element";
            ERROR(msg);
        }

        geometryElement = visualElement->GetElement("geometry");
    } else {
        geometryElement = collisionElement->GetElement("geometry");
    }

    sdf::ElementPtr geometryTypeElement;
    if (!geometryElement) {
        ERROR(modelName + " has no geometryElement");
    }

    if (geometryElement->HasElement("mesh")) {
        geometryTypeElement = geometryElement->GetElement("mesh");
        body = std::move(processMeshElement(modelName + "::" + name,
                                            geometryTypeElement,
                                            linkWorldPose,
                                            collisionElement,
                                            visualElement,
                                            enabled));
    } else if (geometryElement->HasElement("box")) {
        geometryTypeElement = geometryElement->GetElement("box");
        body = std::move(processBoxElement(modelName + "::" + name,
                                           geometryTypeElement,
                                           linkWorldPose,
                                           collisionElement,
                                           visualElement,
                                           enabled));
    } else if (geometryElement->HasElement("sphere")) {
        geometryTypeElement = geometryElement->GetElement("sphere");
        body = std::move(processSphereElement(modelName + "::" + name,
                                              geometryTypeElement,
                                              linkWorldPose,
                                              collisionElement,
                                              visualElement,
                                              enabled));
    } else if (geometryElement->HasElement("cylinder")) {
        geometryTypeElement = geometryElement->GetElement("cylinder");
        body = std::move(processCylinderElement(modelName + "::" + name,
                                                geometryTypeElement,
                                                linkWorldPose,
                                                collisionElement,
                                                visualElement,
                                                enabled));
    } else {
        WARNING("SDFParser: Couldn't parse SDF string. It either has an unsupported geometry type, or is not a model");
        return nullptr;
    }

    return std::move(body);
}

VectorBodyUniquePtr SDFEnvironmentParser::parseBodiesFromFile(std::string& filename,
        const std::string& robotName) const
{
    const std::string file = filename;
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(file, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    VectorBodyUniquePtr bodies;
    sdf::ElementPtr rootElement = sdfModel->Root();
    if (rootElement) {
        sdf::ElementPtr worldElement = rootElement->GetElement("world");
        if (worldElement) {
            sdf::ElementPtr modelElement = worldElement->GetElement("model");
            while (modelElement) {
                std::string modelName = modelElement->Get<std::string>("name");
                if (modelName != robotName) {
                    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
                    sdf::ElementPtr linkElement = modelElement->GetElement("link");
                    while (linkElement) {
                        BodyUniquePtr body = std::move(processLinkElement(modelName, poseElement, linkElement));
                        if (body)
                            bodies.push_back(std::move(body));
                        linkElement = linkElement->GetNextElement("link");
                    }
                }

                modelElement = modelElement->GetNextElement("model");
            }

            sdf::ElementPtr stateElement = worldElement->GetElement("state");
            if (stateElement) {
                VectorBodyPtr vectorBodyPtr(bodies.size(), nullptr);
                for (size_t i = 0; i != vectorBodyPtr.size(); ++i) {
                    vectorBodyPtr[i] = bodies[i].get();
                }
                processStateElement(stateElement, robotName, vectorBodyPtr);
            }
        }
    }

    return bodies;
}

bool SDFEnvironmentParser::processStateElement(const sdf::ElementPtr &stateElement, const std::string& robotName, VectorBodyPtr &bodies) const {
    sdf::ElementPtr modelElement = stateElement->GetElement("model");
    while (modelElement) {
        std::string modelName = modelElement->Get<std::string>("name");
        if (modelName != robotName) {
            sdf::ElementPtr modelPoseElement = modelElement->GetElement("pose");
            sdf::ElementPtr linkElement = modelElement->GetElement("link");
            while (linkElement) {
                geometric::Pose parentModelPose = processPoseElement(modelPoseElement);
                processBodyState(modelName, parentModelPose, linkElement, bodies);
                linkElement = linkElement->GetNextElement("link");
            }
        }

        modelElement = modelElement->GetNextElement("model");
    }
    return true;
}

bool SDFEnvironmentParser::processBodyState(const std::string &modelName,
        const geometric::Pose &parentModelPose,
        const sdf::ElementPtr &linkElement,
        VectorBodyPtr &bodies) const {

    std::string linkName = linkElement->Get<std::string>("name");

    sdf::ElementPtr linkPoseElement = linkElement->GetElement("pose");
    if (!linkPoseElement)
        ERROR("Link in state SDF has no pose element");

    geometric::Pose linkPose = processPoseElement(linkPoseElement);

    // Find the body
    for (auto &body : bodies) {
        if (modelName + "::" + linkName == body->getName()) {
            body->setWorldPose(linkPose);
            body->updateCollisionObject();
        }
    }

    return true;
}


VectorFloat SDFEnvironmentParser::toDoubleVec(const VectorString& stringVec) const
{
    VectorFloat FloatTypeVec;
    for (size_t i = 0; i < stringVec.size(); i++) {
        FloatTypeVec.push_back(atof(stringVec[i].c_str()));
    }

    return FloatTypeVec;
}

geometric::Pose SDFEnvironmentParser::processPoseElement(const sdf::ElementPtr& poseElement) const
{
    geometric::Pose pose;
    if (poseElement) {
        if (poseElement->GetValue()) {
            std::string poseStr = poseElement->GetValue()->GetAsString();
            std::vector<std::string> elems;
            split(poseStr, ' ', elems);
            VectorFloat poseVec = toDoubleVec(elems);
            pose = geometric::Pose(GZPose(poseVec[0], poseVec[1], poseVec[2], poseVec[3], poseVec[4], poseVec[5]));
            return pose;
        } else {
            oppt::WARNING("Pose of SDF element is malformed!");
        }
    }

    return pose;
}

std::pair<GeometrySharedPtr, VectorFloat> SDFEnvironmentParser::processVisualElement(const sdf::ElementPtr& visualElement,
        const geometric::Pose &parentLinkPose) const
{
    GeometrySharedPtr geometry = nullptr;
    VectorFloat color( {0.5, 0.5, 0.5, 1.0});
    if (!visualElement) {
        WARNING("Link has no visual element");
        std::pair<GeometrySharedPtr, VectorFloat> p = {nullptr, color};
        return p;
    }

    geometric::Pose visualPose(parentLinkPose);
    sdf::ElementPtr poseElement = visualElement->GetElement("pose");
    if (poseElement) {
        std::string poseString = poseElement->GetValue()->GetAsString();
        VectorString poseStringElems;
        split(poseString, ' ', poseStringElems);
        visualPose.position.x() = atof(poseStringElems[0].c_str());
        visualPose.position.y() = atof(poseStringElems[1].c_str());
        visualPose.position.z() = atof(poseStringElems[2].c_str());

        Vector3f euler;
        euler.x() = atof(poseStringElems[3].c_str());
        euler.y() = atof(poseStringElems[4].c_str());
        euler.z() = atof(poseStringElems[5].c_str());

        visualPose = geometric::Pose(geometric::Pose(visualPose.position, euler).toGZPose() + parentLinkPose.toGZPose());
    }

    sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
    if (!geometryElement) {
        WARNING("Visual element has no geometry");
        std::pair<GeometrySharedPtr, VectorFloat> p = {nullptr, color};
        return p;
    }
    sdf::ElementPtr geometryTypeElement;
    if (geometryElement->HasElement("mesh")) {
        geometry = makeMeshGeometry(geometryElement, visualPose);
    } else if (geometryElement->HasElement("box")) {
        geometry = makeBoxGeometry(geometryElement, visualPose);
    } else if (geometryElement->HasElement("sphere")) {
        geometry = makeSphereGeometry(geometryElement, visualPose);
    } else if (geometryElement->HasElement("cylinder")) {
        geometry = makeCylinderGeometry(geometryElement, visualPose);
    } else {
        WARNING("SDFParser: Couldn't parse SDF string. It either has an unsupported geometry type, or is not a model");
        //return nullptr;
    }

    if (visualElement->HasElement("material")) {
        sdf::ElementPtr materialElem = visualElement->GetElement("material");
        if (materialElem->HasElement("ambient")) {
            sdf::ElementPtr ambientElem = materialElem->GetElement("ambient");
            std::string colorString = ambientElem->GetValue()->GetAsString();
            VectorString pieces;
            split(colorString, ' ', pieces);
            color[0] = atof(pieces[0].c_str());
            color[1] = atof(pieces[1].c_str());
            color[2] = atof(pieces[2].c_str());
            color[3] = atof(pieces[3].c_str());
        }
    }

    std::pair<GeometrySharedPtr, VectorFloat> p = {geometry, color};
    return p;
}

geometric::Pose SDFEnvironmentParser::getCollisionPose(const sdf::ElementPtr &collisionElement, const geometric::Pose &parentLinkPose) const {
    sdf::ElementPtr collisionPoseElem = collisionElement->GetElement("pose");
    if (!collisionPoseElem)
        return parentLinkPose;
    std::string collisionPoseString = collisionPoseElem->GetValue()->GetAsString();
    VectorString collisionPoseElems;
    split(collisionPoseString, ' ', collisionPoseElems);
    VectorFloat collisionPoseVec = toDoubleVec(collisionPoseElems);

    geometric::Pose collisionPoseLocal(collisionPoseVec[0], collisionPoseVec[1], collisionPoseVec[2], collisionPoseVec[3], collisionPoseVec[4], collisionPoseVec[5]);
    geometric::Pose collisionPose = geometric::Pose(collisionPoseLocal.toGZPose() + parentLinkPose.toGZPose());
    return collisionPose;
}


BodyUniquePtr SDFEnvironmentParser::processMeshElement(const std::string& name,
        const sdf::ElementPtr& meshElement,
        const geometric::Pose& parentLinkPose,
        const sdf::ElementPtr& collisionElement,
        const sdf::ElementPtr& visualElement,
        bool& enabled) const
{
    if (meshElement->HasAttribute("scale")) {

    }

    std::string meshUri = meshElement->GetElement("uri")->GetValue()->GetAsString();
    std::string scaleStr = meshElement->GetElement("scale")->GetValue()->GetAsString();
    std::vector<std::string> scaleVecElemes;
    split(scaleStr, ' ', scaleVecElemes);
    VectorFloat scale( {1.0, 1.0, 1.0});
    if (scaleVecElemes.size() == 3) {
        scale[0] = atof(scaleVecElemes[0].c_str());
        scale[1] = atof(scaleVecElemes[1].c_str());
        scale[2] = atof(scaleVecElemes[2].c_str());
    }

    VectorString meshURIs( {meshUri});
    for (auto & meshUri : meshURIs) {
        if (!oppt::resources::FileExists(meshUri))
            ERROR("File '" + meshUri + "' doesn't exist");
    }

    geometric::Pose collisionPose = getCollisionPose(collisionElement, parentLinkPose);
    auto visualElem = processVisualElement(visualElement, parentLinkPose);
    if (!(visualElem.first))
        WARNING("Visual element is null");
    bool fromFile = false;
    BodyUniquePtr body(new oppt::MeshBody(name, meshURIs, parentLinkPose, scale, fromFile));
    body->setEnabled(enabled);
    body->initVisualGeometry(visualElem.first);
    body->setColor(visualElem.second);
    body->getCollisionGeometry()->setWorldPose(collisionPose);
    //body->getVisualGeometry()->setParentWorldPose(parentLinkPose);
    return std::move(body);
}

BodyUniquePtr SDFEnvironmentParser::processSphereElement(const std::string& name,
        const sdf::ElementPtr& sphereElement,
        const geometric::Pose& parentLinkPose,
        const sdf::ElementPtr& collisionElement,
        const sdf::ElementPtr& visualElement,
        bool& enabled) const
{
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement)
        ERROR("Sphere has no radius element");

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    geometric::Pose collisionWorldPose = getCollisionPose(collisionElement, parentLinkPose);
    auto visualElem = processVisualElement(visualElement, parentLinkPose);
    BodyUniquePtr body(new SphereBody(name, parentLinkPose, radius));
    body->setEnabled(enabled);
    body->initVisualGeometry(visualElem.first);
    body->setColor(visualElem.second);
    body->getCollisionGeometry()->setWorldPose(collisionWorldPose);
    //body->getVisualGeometry()->setParentWorldPose(parentLinkPose);
    //body->updateCollisionObject();
    return std::move(body);
}

BodyUniquePtr SDFEnvironmentParser::processCylinderElement(const std::string& name,
        const sdf::ElementPtr& cylinderElement,
        const geometric::Pose& parentLinkPose,
        const sdf::ElementPtr& collisionElement,
        const sdf::ElementPtr& visualElement,
        bool& enabled) const
{
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement)
        ERROR("Cylinder has no radius element");

    if (!lengthElement)
        ERROR("Cylinder has no length element");

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    geometric::Pose collisionWorldPose = getCollisionPose(collisionElement, parentLinkPose);
    auto visualElem = processVisualElement(visualElement, parentLinkPose);
    BodyUniquePtr body(new CylinderBody(name, parentLinkPose, radius, length));
    body->setEnabled(enabled);
    body->initVisualGeometry(visualElem.first);
    body->setColor(visualElem.second);
    body->getCollisionGeometry()->setWorldPose(collisionWorldPose);
    return std::move(body);
}

BodyUniquePtr SDFEnvironmentParser::processBoxElement(const std::string& name,
        const sdf::ElementPtr& boxElement,
        const geometric::Pose& parentLinkPose,
        const sdf::ElementPtr& collisionElement,
        const sdf::ElementPtr& visualElement,
        bool& enabled) const
{
    sdf::ElementPtr sizeElement = boxElement->GetElement("size");
    if (!sizeElement)
        ERROR("Box has no size element");


    std::string sizeStr = sizeElement->GetValue()->GetAsString();
    VectorString sizeElems;
    split(sizeStr, ' ', sizeElems);
    if (sizeElems.size() != 3) 
        ERROR("Size of box has incorrect number of values");        

    FloatType sizeX = atof(sizeElems[0].c_str());
    FloatType sizeY = atof(sizeElems[1].c_str());
    FloatType sizeZ = atof(sizeElems[2].c_str());
    VectorFloat dimensions( {sizeX, sizeY, sizeZ});
    geometric::Pose collisionWorldPose = getCollisionPose(collisionElement, parentLinkPose);
    auto visualElem = processVisualElement(visualElement, parentLinkPose);
    BodyUniquePtr body(new oppt::BoxBody(name,
                                         parentLinkPose,
                                         dimensions));
    body->setEnabled(enabled);
    body->initVisualGeometry(visualElem.first);
    body->setColor(visualElem.second);
    body->getCollisionGeometry()->setWorldPose(collisionWorldPose);
    //body->getVisualGeometry()->setParentWorldPose(parentLinkPose);
    return std::move(body);
}

GeometrySharedPtr SDFEnvironmentParser::makeBoxGeometry(const sdf::ElementPtr &geometryElement,
        const geometric::Pose &visualPose) const {
    sdf::ElementPtr boxElement = geometryElement->GetElement("box");
    sdf::ElementPtr sizeElement = boxElement->GetElement("size");
    if (!sizeElement)
        ERROR("Box has no size element");

    std::string sizeStr = sizeElement->GetValue()->GetAsString();
    VectorString sizeElems;
    split(sizeStr, ' ', sizeElems);
    if (sizeElems.size() != 3)
        ERROR("Size of box has incorrect number of values");

    VectorFloat dimensions({atof(sizeElems[0].c_str()),
                            atof(sizeElems[1].c_str()),
                            atof(sizeElems[2].c_str())
                           });
    GeometrySharedPtr geom(new geometric::Box("_default_", dimensions, visualPose));
    return geom;
}

GeometrySharedPtr SDFEnvironmentParser::makeCylinderGeometry(const sdf::ElementPtr &geometryElement,
        const geometric::Pose &visualPose) const {
    sdf::ElementPtr cylinderElement = geometryElement->GetElement("cylinder");
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement)
        ERROR("Cylinder has no radius element");


    if (!lengthElement)
        ERROR("Cylinder has no length element");

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    GeometrySharedPtr geom(new geometric::Cylinder("_default_", radius, length, visualPose));
    return geom;
}

GeometrySharedPtr SDFEnvironmentParser::makeSphereGeometry(const sdf::ElementPtr &geometryElement,
        const geometric::Pose &visualPose) const {
    sdf::ElementPtr sphereElement = geometryElement->GetElement("sphere");
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement)
        ERROR("Sphere has no radius element");
    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    GeometrySharedPtr geom(new geometric::Sphere("_default_", radius, visualPose));
    return geom;
}

GeometrySharedPtr SDFEnvironmentParser::makeMeshGeometry(const sdf::ElementPtr &geometryElement,
        const geometric::Pose &visualPose) const {
    sdf::ElementPtr meshElement = geometryElement->GetElement("mesh");
    sdf::ElementPtr uriElement = meshElement->GetElement("uri");
    if (!uriElement)
        ERROR("NO URI ELEMENT");
    std::string uri = uriElement->GetValue()->GetAsString().c_str();
    if (!(oppt::resources::FileExists(uri)))
        WARNING("Mesh file '" + uri + "' couldn't be found");
    std::string meshFile = oppt::resources::FindFile(uri);

    VectorFloat scale({1.0, 1.0, 1.0});

    sdf::ElementPtr scaleElement = meshElement->GetElement("scale");
    if (scaleElement) {
        std::string scaleString = scaleElement->GetValue()->GetAsString();
        VectorString scaleElems;
        split(scaleString, ' ', scaleElems);
        if (scaleElems.size() == 3) {
            scale[0] = atof(scaleElems[0].c_str());
            scale[1] = atof(scaleElems[1].c_str());
            scale[2] = atof(scaleElems[2].c_str());
        }
    }

    GeometrySharedPtr geom(new geometric::Mesh("_default_", meshFile, visualPose, scale));
    return geom;
}


}
