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
#include "include/Body.hpp"
#include "oppt/opptCore/resources/resources.hpp"
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
    sdf::ElementPtr collisionElement =
        linkElement->GetElement("collision");
    sdf::ElementPtr visualElement =
        linkElement->GetElement("visual");
    sdf::ElementPtr linkPoseElement = linkElement->GetElement("pose");
    geometric::Pose linkWorldPose(processPoseElement(linkPoseElement).toGZPose() +
                                  processPoseElement(modelPoseElement).toGZPose());
    std::string linkName = modelName + "::" + linkElement->Get<std::string>("name");
    BodyUniquePtr body(new BodyImpl(linkName, linkWorldPose));
    while (collisionElement) {
        std::string collisionName = collisionElement->Get<std::string>("name");
        if (collisionName != "__default__") {
            GeometryUniquePtr collisionGeometry = std::move(makeCollisionGeometry(collisionElement, linkName, linkWorldPose));
            if (!collisionGeometry)
                ERROR("Collision geometry is null");
            body->addCollisionGeometry(std::move(collisionGeometry));
        }

        collisionElement = collisionElement->GetNextElement("collision");
    }

    while (visualElement) {
        std::string visualName = visualElement->Get<std::string>("name");
        if (visualName != "__default__") {
            GeometryUniquePtr visualGeometry = std::move(makeVisualGeometry(visualElement, linkName, linkWorldPose));
            if (!visualGeometry)
                ERROR("Visual geometry is null");
            body->addVisualGeometry(std::move(visualGeometry));
        }
        visualElement = visualElement->GetNextElement("visual");
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
            body->updateCollisionObjects();
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

GeometryUniquePtr SDFEnvironmentParser::makeVisualGeometry(const sdf::ElementPtr &visualElement,
        const std::string &parentName,
        const geometric::Pose& parentLinkPose) const {
    sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
    if (!geometryElement) {
        ERROR(visualElement->Get<std::string>("name") + " has no geometryElement");
    }

    std::string visualName = visualElement->Get<std::string>("name");
    visualName = parentName + "::" + visualName;
    geometric::Pose collisionPose = getCollisionPose(visualElement, parentLinkPose);
    GeometryUniquePtr visualGeometry = nullptr;
    if (geometryElement->HasElement("mesh")) {
        visualGeometry = std::move(makeMeshGeometry(geometryElement, visualName, collisionPose));
    } else if (geometryElement->HasElement("box")) {
        visualGeometry = std::move(makeBoxGeometry(geometryElement, visualName, collisionPose));
    } else if (geometryElement->HasElement("cylinder")) {
        visualGeometry = std::move(makeCylinderGeometry(geometryElement, visualName, collisionPose));
    } else if (geometryElement->HasElement("sphere")) {
        visualGeometry = std::move(makeSphereGeometry(geometryElement, visualName, collisionPose));
    }

    VectorFloat color( {0.5, 0.5, 0.5, 1.0});
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

    visualGeometry->setColor(color);
    return std::move(visualGeometry);

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


GeometryUniquePtr SDFEnvironmentParser::makeCollisionGeometry(const sdf::ElementPtr &collisionElement,
        const std::string &parentName,
        const geometric::Pose& parentLinkPose) const {
    sdf::ElementPtr geometryElement = collisionElement->GetElement("geometry");
    if (!geometryElement) {
        ERROR(collisionElement->Get<std::string>("name") + " has no geometryElement");
    }

    std::string collisionName = collisionElement->Get<std::string>("name");
    collisionName = parentName + "::" + collisionName;
    geometric::Pose collisionPose = getCollisionPose(collisionElement, parentLinkPose);
    GeometryUniquePtr collisionGeometry = nullptr;
    if (geometryElement->HasElement("mesh")) {
        collisionGeometry = std::move(makeMeshGeometry(geometryElement, collisionName, collisionPose));
    } else if (geometryElement->HasElement("box")) {
        collisionGeometry = std::move(makeBoxGeometry(geometryElement, collisionName, collisionPose));
    } else if (geometryElement->HasElement("cylinder")) {
        collisionGeometry = std::move(makeCylinderGeometry(geometryElement, collisionName, collisionPose));
    } else if (geometryElement->HasElement("sphere")) {
        collisionGeometry = std::move(makeSphereGeometry(geometryElement, collisionName, collisionPose));
    }

    return std::move(collisionGeometry);
}

GeometryUniquePtr SDFEnvironmentParser::makeBoxGeometry(const sdf::ElementPtr &geometryElement,
        const std::string &parentName,
        const geometric::Pose &parentPose) const {
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
    GeometryUniquePtr geometry(new geometric::Box(parentName, dimensions, parentPose));
    return std::move(geometry);
}

GeometryUniquePtr SDFEnvironmentParser::makeMeshGeometry(const sdf::ElementPtr &geometryElement,
        const std::string &parentName,
        const geometric::Pose &parentPose) const {
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

    GeometryUniquePtr geometry(new geometric::Mesh(parentName,
                               meshFile,
                               parentPose,
                               scale));
    return std::move(geometry);
}

GeometryUniquePtr SDFEnvironmentParser::makeCylinderGeometry(const sdf::ElementPtr &geometryElement,
        const std::string &parentName,
        const geometric::Pose &parentPose) const {
    sdf::ElementPtr cylinderElement = geometryElement->GetElement("cylinder");
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement)
        ERROR("Cylinder has no radius element");


    if (!lengthElement)
        ERROR("Cylinder has no length element");

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    GeometryUniquePtr geometry(new geometric::Cylinder(parentName,
                               radius,
                               length,
                               parentPose));
    return std::move(geometry);
}

GeometryUniquePtr SDFEnvironmentParser::makeSphereGeometry(const sdf::ElementPtr &geometryElement,
        const std::string &parentName,
        const geometric::Pose &parentPose) const {
    sdf::ElementPtr sphereElement = geometryElement->GetElement("sphere");
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement)
        ERROR("Sphere has no radius element");
    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    GeometryUniquePtr geometry(new geometric::Sphere(parentName,
                               radius,
                               parentPose));
    return std::move(geometry);
}




}
