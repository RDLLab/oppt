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
#ifndef __SDF_TO_URDF_HPP__
#define __SDF_TO_URDF_HPP__
#include "utils.hpp"
#include "logging.hpp"
#include "includes.hpp"
#include "typedefs.hpp"
#include "MathUtils.hpp"

namespace oppt
{

class TransformationNode;
typedef std::shared_ptr<TransformationNode> NodePtr;

class TransformationTree;
class TransformationNode
{
    friend class TransformationTree;
public:
    TransformationNode(std::string frame):
        frame_(frame),
        parent_(nullptr),
        children_(),
        childs_() {

    }

    std::string getFrame() const {
        return frame_;
    }

    TransformationNode* getParent() const {
        return parent_;
    }

    void addChild(std::string frame, Matrix4f& transformation) {
        NodePtr node(new TransformationNode(frame));
        children_[node] = transformation;
        node->setParent_(this);
        childs_.push_back(node);
    }

    void addChild(NodePtr child, Matrix4f& transformation) {
        children_[child] = transformation;
        child->setParent_(this);
        childs_.push_back(child);
    }

    /**
     * Recusively gets the transformation to a child frame, starting from the frame of this node
     * */
    Matrix4f getTransformationTo(std::string frame, Matrix4f& currentTransformation) {
        Matrix4f newTransform = Matrixdf::Identity(4, 4);
        if (frame == frame_) {
            return newTransform;
        }
        newTransform.setZero();
        for (auto & child : children_) {
            newTransform = currentTransformation * child.second;
            if (child.first->getFrame() == frame) {
                return newTransform;
            }

            newTransform = child.first->getTransformationTo(frame, newTransform);
            if (!newTransform.isZero()) {
                return newTransform;
            }
        }

        return newTransform;
    }

private:
    NodePtr getNode_(std::string frame) {
        NodePtr node;
        for (auto & child : children_) {
            if (child.first->getFrame() == frame) {
                return child.first;
            }

            node = child.first->getNode_(frame);
            if (node) {
                return node;
            }
        }

        return node;
    }

    void setParent_(TransformationNode* parent) {
        parent_ = parent;
    }

    std::string frame_;

    TransformationNode* parent_;

    std::vector<NodePtr> childs_;

    // The children of this node on the transformations to them
    std::map<NodePtr, Matrixdf> children_;

};

typedef std::pair<NodePtr, NodePtr> NodePair;
typedef std::map<NodePair, Matrixdf> TransformationMap;


class TransformationTree
{
public:
    TransformationTree():
        root_(nullptr) {

    }

    void setRoot(NodePtr& root) {
        root_ = root;
    }

    NodePtr getRoot() {
        return root_;
    }

    /**
     * Recursively gets a child node representing frame 'frame'
     */
    NodePtr getNode(std::string frame) {
        if (frame == root_->getFrame()) {
            return root_;
        }

        return root_->getNode_(frame);
    }

    bool isChildOf(std::string frame1, std::string frame2) {
        NodePtr node1 = getNode(frame1);
        NodePtr node2 = getNode(frame2);
        if (!node1 || !node2) {
            ERROR("TFTree: isChildOf: Node1 or node2 two doesn't exist in tree");
        }

        node2 = node1->getNode_(frame2);
        if (node2) {
            return true;
        }

        return false;
    }

    Matrix4f getTransformationTo(std::string frame1, std::string frame2) {
        if (!isChildOf(frame1, frame2)) {
            std::string errMsg = "TFTree: '";
            errMsg += frame2;
            errMsg += "' is no child of '";
            errMsg += frame1;
            errMsg += "' ";
            ERROR(errMsg);
        }

        NodePtr node1 = getNode(frame1);
        Matrix4f currentTransform = Matrixdf::Identity(4, 4);
        return node1->getTransformationTo(frame2, currentTransform);

    }

    Matrix4f getTransformationFromParent(std::string frame) {
        Matrix4f currentTransform = Matrixdf::Identity(4, 4);
        NodePtr currentNode = getNode(frame);
        if (!currentNode) {
            std::string errString = "TFTree: Node '";
            errString += frame;
            errString += "' doesn't exist in tree";
            ERROR(errString);
        }
        return currentNode->getParent()->getTransformationTo(frame, currentTransform);
    }

private:
    NodePtr root_;

};


class SDFToURDFConverter
{
public:
    SDFToURDFConverter():
        robotXml_(nullptr),
        sdfPath_("") {}

    std::string parseFromFile(std::string& file) {
        if (!fileExists(file)) {
            ERROR("File does not exist");
        }

        VectorString pathElems;
        split(file, '/', pathElems);
        for (size_t i = 1; i < pathElems.size() - 1; i++) {
            sdfPath_ += "/";
            sdfPath_ += pathElems[i];
        }

        std::string sdfString = prepareSDFString(file);
	if (sdfString.empty()) {
	    return sdfString;
	}

        TiXmlDocument xmlDoc;
        xmlDoc.Parse(sdfString.c_str());
        //xmlDoc.LoadFile(file);
        initTfTree(xmlDoc);
        std::string urdfString = "<?xml version=\"1.0\" ?>";
        urdfString += "<robot name='";
        urdfString += getRobotName(xmlDoc);
        urdfString += "'>";
        urdfString += processLinks(xmlDoc);
        urdfString += processJoints(xmlDoc);
        urdfString += "</robot>";        
        return urdfString;
    }

private:
    std::shared_ptr<TransformationTree> tfTree_;

    TiXmlElement* robotXml_;

    std::string sdfPath_;

    std::string getParentOrChildName(TiXmlElement* jointXml, std::string parentOrChild) {
        std::string parentName = "";
        TiXmlElement* parentXml = jointXml->FirstChildElement(parentOrChild);
        if (parentXml) {
            std::string parentText(parentXml->GetText());
            return parentText;

        }

        return parentName;
    }

    std::string prepareSDFString(std::string& file) {
        TiXmlDocument xmlDoc;
        xmlDoc.LoadFile(file);
        TiXmlElement* sdfXml = xmlDoc.FirstChildElement("sdf");
        TiXmlElement* robotXml = sdfXml->FirstChildElement("model");
        std::string rootLinkName = getRootLinkName(robotXml);
	if (rootLinkName.empty()) {
	    return rootLinkName;
	}

        std::ifstream filef(file.c_str());
        std::stringstream buffer;
        buffer << filef.rdbuf();
        std::string sdfString = buffer.str();

        size_t sdfEndPos = sdfString.rfind("model");
        sdfString.erase(sdfString.begin() + sdfEndPos - 2, sdfString.end());
        sdfString += "\n  <link name=\"_root_link_\" />";
        sdfString += "\n  <joint name=\"_root_to_first_link_\" type=\"fixed\">";
        sdfString += "\n   <child>";
        sdfString += rootLinkName;
        sdfString += "</child>";
        sdfString += "\n   <parent>_root_link_</parent>";
        sdfString += "\n  </joint>";
        sdfString += "\n </model>";
        sdfString += "\n</sdf>";        
        return sdfString;
    }

    NodePtr findRoot() {
        TiXmlElement* jointXml = robotXml_->FirstChildElement("joint");
        VectorString parents;
        VectorString children;
        while (jointXml) {
            parents.push_back(getParentOrChildName(jointXml, "parent"));
            children.push_back(getParentOrChildName(jointXml, "child"));
            jointXml = jointXml->NextSiblingElement("joint");
        }

        for (auto & parent : parents) {
            bool found = false;
            for (auto & child : children) {
                if (parent == child) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                NodePtr rootNode(new TransformationNode(parent));
                return rootNode;
            }
        }


        return nullptr;
    }

    std::string getLinkName(TiXmlElement* linkXml) {
        std::string returnString(linkXml->Attribute("name"));
        return returnString;
    }

    Matrix4f getPoseRelativeToParent(TiXmlElement* poseXml, NodePtr& currentNode, bool assumeEmptyStringIsParent = false) {
        if (!currentNode) {
            ERROR("SDFToURDF: Current node is null");
        }
        Matrix4f poseRelativeTo = getPoseMatrix(poseXml);
        std::string childPoseRelativeTo(poseXml->Attribute("frame"));
        if (childPoseRelativeTo == "") {
            if (assumeEmptyStringIsParent) {
                childPoseRelativeTo = currentNode->getFrame();
            } else {
                childPoseRelativeTo = tfTree_->getRoot()->getFrame();
            }
        }

        Matrix4f currentPoseRelativeTo = Matrixdf::Identity(4, 4);
        NodePtr relativeNode = tfTree_->getNode(childPoseRelativeTo);
        if (!relativeNode) {
            ERROR("SDFToURDF: Current node is null");
        }
        currentPoseRelativeTo =
            relativeNode->getTransformationTo(currentNode->getFrame(), currentPoseRelativeTo);
        return currentPoseRelativeTo.inverse() * poseRelativeTo;
    }

    void parseChildNodes(NodePtr currentNode) {
        TiXmlElement* jointXml = robotXml_->FirstChildElement("joint");
        VectorString children;
        while (jointXml) {
            std::string parentName = getParentOrChildName(jointXml, "parent");
            if (parentName == currentNode->getFrame()) {
                children.push_back(getParentOrChildName(jointXml, "child"));
            }

            jointXml = jointXml->NextSiblingElement("joint");
        }

        for (auto & child : children) {
            TiXmlElement* childLinkXml = getLinkXml(child);
            TiXmlElement* childPoseXml = childLinkXml->FirstChildElement("pose");
            Matrix4f childPoseRelativeToParent = Matrixdf::Identity(4, 4);
            if (childPoseXml) {
                childPoseRelativeToParent = getPoseRelativeToParent(childPoseXml, currentNode);
            }
            
            NodePtr childNode(new TransformationNode(child));            
            currentNode->addChild(childNode, childPoseRelativeToParent);
            NodePtr nextNode = tfTree_->getNode(child);
            if (!nextNode) {
                ERROR("next node is null");
            }
            parseChildNodes(nextNode);

        }
    }

    std::string getRootLinkName(TiXmlElement* robotXml = nullptr) {
        if (!robotXml) {
            robotXml = robotXml_;
        }        
        
	if (!robotXml || !robotXml->FirstChild("joint")) {
	    std::string returnString("");
	    return returnString;
	}
        
        TiXmlElement* jointXml = robotXml->FirstChildElement("joint");
        VectorString parents;
        VectorString children;
        while (jointXml) {
            parents.push_back(getParentOrChildName(jointXml, "parent"));
            children.push_back(getParentOrChildName(jointXml, "child"));
            jointXml = jointXml->NextSiblingElement("joint");
        }

        for (auto & parent : parents) {
            bool found = false;
            for (auto & child : children) {
                if (parent == child) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return parent;
            }
        }
    }

    Matrix4f getRootLinkPose() {
        std::string rootLinkName = getRootLinkName();
        TiXmlElement* linkXml = robotXml_->FirstChildElement("link");
        while (linkXml) {
            std::string linkName(linkXml->Attribute("name"));
            if (linkName == rootLinkName) {
                TiXmlElement* poseXml = linkXml->FirstChildElement("pose");
                if (poseXml) {
                    Matrix4f rootLinkPose = getPoseMatrix(poseXml);
                    return rootLinkPose;
                }

            }

            linkXml = linkXml->NextSiblingElement("link");
        }

        return Matrixdf::Identity(4, 4);
    }

    void initTfTree(TiXmlDocument& xmlDoc) {
        tfTree_ = std::make_shared<TransformationTree>();

        TiXmlElement* sdfXml = xmlDoc.FirstChildElement("sdf");
        if (!sdfXml) {
            ERROR("File is no SDF file");
        }

        robotXml_ = sdfXml->FirstChildElement("model");
        NodePtr root = findRoot();        
        tfTree_->setRoot(root);
        Matrix4f currentTransform = Matrixdf::Identity(4, 4);
        parseChildNodes(root);
    }

    VectorFloat toDoubleVec(VectorString& stringVec) {
        VectorFloat FloatTypeVec;
        for (size_t i = 0; i < stringVec.size(); i++) {
            FloatTypeVec.push_back(atof(stringVec[i].c_str()));
        }

        return FloatTypeVec;
    }

    std::string getRobotName(TiXmlDocument& xmlDoc) {
        TiXmlElement* sdfXml = xmlDoc.FirstChildElement("sdf");
        if (!sdfXml) {
            ERROR("File is no SDF file");
        }

        TiXmlElement* robotXml = sdfXml->FirstChildElement("model");
        if (!robotXml) {
            ERROR("File has no model");
        }
        const char* nameChar = robotXml->Attribute("name");

        std::string robotName(nameChar);
        return robotName;
    }

    std::string processInertiaElem(TiXmlElement* inertiaXml) {
        std::string returnString = "<inertia ";
        std::string val;
        TiXmlElement* valE = inertiaXml->FirstChildElement("ixx");

        val = std::string(valE->GetText());
        returnString += "ixx=" + val + " ";

        valE = inertiaXml->FirstChildElement("ixy");
        val = std::string(valE->GetText());
        returnString += "ixy=" + val + " ";

        valE = inertiaXml->FirstChildElement("ixz");
        val = std::string(valE->GetText());
        returnString += "ixz=" + val + " ";

        valE = inertiaXml->FirstChildElement("iyy");
        val = std::string(valE->GetText());
        returnString += "iyy=" + val + " ";

        valE = inertiaXml->FirstChildElement("iyz");
        val = std::string(valE->GetText());
        returnString += "iyz=" + val + " ";

        valE = inertiaXml->FirstChildElement("izz");
        val = std::string(valE->GetText());
        returnString += "izz=" + val;

        returnString += " />";
        return returnString;
    }

    std::string matrixToOriginStr(Matrix4f& mat) {
        std::string returnString = "<origin xyz=\"";
        std::string xyzString = std::to_string(mat(0, 3)) + " ";
        xyzString += std::to_string(mat(1, 3)) + " ";
        xyzString += std::to_string(mat(2, 3));
        returnString += xyzString + "\" rpy=\"";
        Matrix3f rotPart = mat.block<3, 3>(0, 0);
        Vector3f eulerAngles = rotPart.eulerAngles(0, 1, 2);
        returnString += std::to_string(eulerAngles[0]) + " ";
        returnString += std::to_string(eulerAngles[1]) + " ";
        returnString += std::to_string(eulerAngles[2]);
        returnString += "\" />";
        return returnString;
    }

    std::string processUri(std::string& uri) {
        size_t loc = 0;
        std::string filePrefix = "file://";
        if (uri.find("package://") != std::string::npos) {
            loc = 9;
        } else if (uri.find("file://") != std::string::npos) {
            loc = 6;
        } else {
            ERROR("Unsupported uri formatting. Make sure it's of the form 'file://...' or 'package://...'");
        }

        // Try out absolute path first
        std::string absolutePath = uri.erase(0, loc);
        if (fileExists(absolutePath)) {
            uri = filePrefix + absolutePath;
            return uri;
        }

        // Now try relative path (relative to SDF file location)
        absolutePath = sdfPath_ + absolutePath;
        if (fileExists(absolutePath)) {
            absolutePath = filePrefix + absolutePath;
            return absolutePath;
        }

        std::string errString = "Mesh file '";
        errString += absolutePath + "' doesn't exist";
        ERROR(errString);
    }

    std::string processGeometryElem(TiXmlElement* geometryXml) {
        std::string returnString = "";
        if (geometryXml) {
            returnString += "<geometry>";
            TiXmlElement* boxXml = geometryXml->FirstChildElement("box");
            TiXmlElement* meshXml = geometryXml->FirstChildElement("mesh");
            TiXmlElement* sphereXml = geometryXml->FirstChildElement("sphere");
            if (boxXml) {
                TiXmlElement* sizeXml = boxXml->FirstChildElement("size");
                if (sizeXml) {
                    returnString += "<box size=\"";
                    std::string sizeText(sizeXml->GetText());
                    returnString += sizeText;
                    returnString += "\" />";
                }
            }

            if (meshXml) {
                returnString += "<mesh ";
                TiXmlElement* scaleXml = meshXml->FirstChildElement("scale");
                TiXmlElement* uriXml = meshXml->FirstChildElement("uri");
                if (scaleXml) {
                    returnString += "scale=\"";
                    std::string scaleText(scaleXml->GetText());
                    returnString += scaleText + "\" ";
                }

                if (uriXml) {
                    returnString += "filename=\"";
                    std::string uriText(uriXml->GetText());
                    uriText = processUri(uriText);
                    returnString += uriText + "\" ";
                }

                returnString += " />";
            }

            if (sphereXml) {

            }

            returnString += "</geometry>";

        }

        return returnString;
    }

    std::string processMaterialElem(TiXmlElement* materialXml) {
        std::string returnString = "";
        if (materialXml) {
            TiXmlElement* scriptXml = materialXml->FirstChildElement("script");
            bool hasScript = false;
            if (scriptXml) {
                hasScript = true;
                TiXmlElement* nameXml = scriptXml->FirstChildElement("name");
                if (nameXml) {
                    returnString += "<material name=\"";
                    std::string nameString(nameXml->GetText());
                    returnString += nameString;
                    returnString += "\" />";
                }
            }

            if (!hasScript) {
                TiXmlElement* ambientXml = materialXml->FirstChildElement("ambient");
                if (ambientXml) {
                    returnString += "<material name=''>";
                    returnString += "<color rgba='";
                    std::string colorString(ambientXml->GetText());
                    returnString += colorString;
                    returnString += "' /></material>";
                }
            }
        }

        return returnString;
    }

    std::string processCollisionOrVisualElem(TiXmlElement* linkXml, std::string typeString) {
        std::string returnString = "";
        TiXmlElement* typeXml = linkXml->FirstChildElement(typeString);
        while (typeXml) {
            returnString += "<" + typeString + ">";
            TiXmlElement* poseXml = typeXml->FirstChildElement("pose");
            std::string linkName(linkXml->Attribute("name"));
            NodePtr node = tfTree_->getNode(linkName);
            Matrix4f poseRelativeToLink(getPoseRelativeToParent(poseXml, node, true));
            returnString += matrixToOriginStr(poseRelativeToLink);
            TiXmlElement* geometryXml = typeXml->FirstChildElement("geometry");
            returnString += processGeometryElem(geometryXml);
            TiXmlElement* materialXml = typeXml->FirstChildElement("material");
            returnString += processMaterialElem(materialXml);
            returnString += "</" + typeString + ">";
            typeXml = typeXml->NextSiblingElement("collision");
        }

        return returnString;
    }

    std::string processInertialElem(TiXmlElement* linkXml) {
        std::string returnString = "";
        TiXmlElement* inertialXml = linkXml->FirstChildElement("inertial");
        if (inertialXml) {
            returnString += "<inertial>";
            TiXmlElement* massXml = inertialXml->FirstChildElement("mass");
            if (massXml) {
                returnString += "<mass value=\"";
                std::string massString(massXml->GetText());
                returnString += massString;
                returnString += "\" />";
            }

            TiXmlElement* inertiaXml = inertialXml->FirstChildElement("inertia");
            if (inertiaXml) {
                returnString += processInertiaElem(inertiaXml);
            }

            TiXmlElement* poseXml = inertialXml->FirstChildElement("pose");
            if (poseXml) {
                //Matrix4f inertiaPose = getPoseMatrix(poseXml);
                std::string linkName(linkXml->Attribute("name"));                
                NodePtr node = tfTree_->getNode(linkName);
                Matrix4f poseRelativeToLink(getPoseRelativeToParent(poseXml, node, true));
                returnString += matrixToOriginStr(poseRelativeToLink);
            }

            returnString += "</inertial>";
        }
        return returnString;
    }

    Matrix4f getPoseMatrix(TiXmlElement* poseXml) {
        if (poseXml) {
            std::string poseStr(poseXml->GetText());
            VectorString poseVecStr;
            split(poseStr, ' ', poseVecStr);
            VectorFloat poseVec = toDoubleVec(poseVecStr);
            Matrix4f rotX = getRotationMatrixX(poseVec[3]);
            Matrix4f rotY = getRotationMatrixY(poseVec[4]);
            Matrix4f rotZ = getRotationMatrixZ(poseVec[5]);
            Matrix4f transMatrix = getTranslationMatrix(poseVec[0], poseVec[1], poseVec[2]);
            Matrix4f res = transMatrix * rotX * rotY * rotZ;
            return res;
        }

        Matrix4f res = Matrixdf::Identity(4, 4);
        return res;
    }

    void processTF(TiXmlElement* rootLinkXml) {
        tfTree_ = std::make_shared<TransformationTree>();
    }

    std::string processLinks(TiXmlDocument& xmlDoc) {
        std::string returnString = "";
        TiXmlElement* sdfXml = xmlDoc.FirstChildElement("sdf");
        if (!sdfXml) {
            ERROR("File is no SDF file");
        }

        robotXml_ = sdfXml->FirstChildElement("model");
        TiXmlElement* linkXml = robotXml_->FirstChildElement("link");
        if (!linkXml) {
            ERROR("Robot has no links");
        }

        while (linkXml) {
            returnString += "<link name='";
            std::string linkName(linkXml->Attribute("name"));
            returnString += linkName + "'>";
            returnString += processInertialElem(linkXml);
            returnString += processCollisionOrVisualElem(linkXml, "collision");
            returnString += processCollisionOrVisualElem(linkXml, "visual");
            returnString += "</link>";
            linkXml = linkXml->NextSiblingElement("link");
        }

        return returnString;
    }

    std::string processParentLink(TiXmlElement* jointXml) {
        std::string returnString = "";
        TiXmlElement* parentXml = jointXml->FirstChildElement("parent");
        if (parentXml) {
            std::string parentText(parentXml->GetText());
            returnString += "\n <parent link='";
            returnString += parentText;
            returnString += "' />";
        }
        return returnString;
    }

    TiXmlElement* getLinkXml(std::string& linkName) {
        TiXmlElement* linkXml = robotXml_->FirstChildElement("link");
        while (linkXml) {
            if (linkXml->Attribute("name") == linkName) {
                return linkXml;
            }
            linkXml = linkXml->NextSiblingElement("link");
        }

        return nullptr;
    }

    std::string processOrigin(std::string& childLink) {
        std::string returnString = "";
        TiXmlElement* linkXml = getLinkXml(childLink);
        if (!linkXml) {
            std::string errorMsg = "child link '" + childLink + "' doesn't exist";
            ERROR(errorMsg);
        }

        Matrix4f poseRelativeToParent = tfTree_->getTransformationFromParent(childLink);
        returnString += matrixToOriginStr(poseRelativeToParent);
        return returnString;
    }

    std::string processChildLink(TiXmlElement* jointXml) {
        std::string returnString = "";
        TiXmlElement* childXml = jointXml->FirstChildElement("child");
        std::string childText = "";
        if (childXml) {
            childText = std::string(childXml->GetText());
            returnString += "\n <child link='";
            returnString += childText;
            returnString += "' /> \n  ";
            returnString += processOrigin(childText);
        }

        return returnString;
    }

    std::string processAxis(TiXmlElement* jointXml) {
        std::string returnString = "";
        TiXmlElement* axisXml = jointXml->FirstChildElement("axis");
        if (axisXml) {
            TiXmlElement* xyzXml = axisXml->FirstChildElement("xyz");
            if (xyzXml) {
                std::string xyzString(xyzXml->GetText());
                returnString += "<axis xyz=\"";
                returnString += xyzString;
                returnString += "\" />";
            }
        }
        return returnString;
    }

    std::string processLimits(TiXmlElement* jointXml) {
        std::string returnString = "";
        TiXmlElement* axisXml = jointXml->FirstChildElement("axis");
        if (axisXml) {
            returnString += "<limit ";
            TiXmlElement* limitXml = axisXml->FirstChildElement("limit");
            if (limitXml) {
                TiXmlElement* lowerXml = limitXml->FirstChildElement("lower");
                TiXmlElement* upperXml = limitXml->FirstChildElement("upper");
                TiXmlElement* effortXml = limitXml->FirstChildElement("effort");
                TiXmlElement* velocityXml = limitXml->FirstChildElement("velocity");

                returnString += "lower=\"";
                std::string lowerString = "0";
                if (lowerXml) {
                    lowerString = lowerXml->GetText();
                }
                returnString += lowerString;
                returnString += "\" ";

                std::string upperString = "0";
                returnString += "upper=\"";
                if (upperXml) {
                    upperString = upperXml->GetText();
                }

                returnString += upperString;
                returnString += "\" ";

                std::string effortString = "0";
                returnString += "effort=\"";
                if (effortXml) {
                    effortString = effortXml->GetText();
                }

                returnString += effortString;
                returnString += "\" ";

                std::string velocityString = "0";
                returnString += "velocity=\"";
                if (velocityXml) {
                    velocityString = velocityXml->GetText();
                }

                returnString += velocityString;
                returnString += "\" ";
            }

            returnString += "/>";
        }

        return returnString;
    }

    std::string processDynamics(TiXmlElement* jointXml) {
        std::string returnString = "";
        TiXmlElement* axisXml = jointXml->FirstChildElement("axis");
        if (axisXml) {
            TiXmlElement* dynamicsXml = axisXml->FirstChildElement("dynamics");
            if (dynamicsXml) {
                returnString += "<dynamics ";
                TiXmlElement* dampingXml = dynamicsXml->FirstChildElement("damping");
                TiXmlElement* frictionXml = dynamicsXml->FirstChildElement("friction");
                if (dampingXml) {
                    std::string dampingString(dampingXml->GetText());
                    returnString += "damping=\"";
                    returnString += dampingString;
                    returnString += "\" ";
                }

                if (frictionXml) {
                    std::string frictionString(frictionXml->GetText());
                    returnString += "friction=\"";
                    returnString += frictionString;
                    returnString += "\" ";
                }
                returnString += "/>";
            }
        }
        return returnString;
    }

    std::string processJoint(TiXmlElement* jointXml) {
        std::string returnString = "\n<joint name='";
        std::string jointName(jointXml->Attribute("name"));
        std::string jointType(jointXml->Attribute("type"));
        returnString += jointName + "' type='";
        returnString += jointType + "'>";
        returnString += processParentLink(jointXml);
        returnString += processChildLink(jointXml);
        returnString += processAxis(jointXml);
        returnString += processLimits(jointXml);
        returnString += processDynamics(jointXml);
        returnString += "\n</joint>";
        return returnString;
    }

    std::string processJoints(TiXmlDocument& xmlDoc) {
        std::string returnString = "";
        TiXmlElement* sdfXml = xmlDoc.FirstChildElement("sdf");
        if (!sdfXml) {
            ERROR("File is no SDF file");
        }

        robotXml_ = sdfXml->FirstChildElement("model");
        TiXmlElement* jointXml = robotXml_->FirstChildElement("joint");
        if (!jointXml) {
            ERROR("Robot has no joints!");
        }

        while (jointXml) {
            returnString += processJoint(jointXml);
            jointXml = jointXml->NextSiblingElement("joint");
        }

        return returnString;
    }

};

}

#endif

