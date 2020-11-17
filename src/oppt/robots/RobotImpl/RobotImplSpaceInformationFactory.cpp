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
#include "oppt/robotHeaders/RobotImpl/RobotImplSpaceInformationFactory.hpp"
#include "oppt/opptCore/StateInformation.hpp"
#include "oppt/opptCore/ActionInformation.hpp"
#include "oppt/opptCore/ObservationInformation.hpp"

namespace oppt
{
namespace SpaceInformationFactory
{
const StateSpaceInformationPtr makeStateSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    std::string cfgPath = robotConfigOptions->configFilePath;
    VectorString robotJointsInWorld;
    VectorString robotLinksInWorld;
    if (gazeboInterface) {
        robotJointsInWorld = gazeboInterface->getRobotJointNames();
        robotLinksInWorld = gazeboInterface->getLinkNames();
    }

    VectorString jointsInConfigFile = robotConfigOptions->jointPositions;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointVelocities.begin(),
                              robotConfigOptions->jointVelocities.end());    

    VectorString linksInConfigFile = robotConfigOptions->linkPoses;
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPoses.begin(),
                             robotConfigOptions->linkPoses.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesLinear.begin(),
                             robotConfigOptions->linkVelocitiesLinear.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesAngular.begin(),
                             robotConfigOptions->linkVelocitiesAngular.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsX.begin(),
                             robotConfigOptions->linkPositionsX.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsY.begin(),
                             robotConfigOptions->linkPositionsY.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsZ.begin(),
                             robotConfigOptions->linkPositionsZ.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsX.begin(),
                             robotConfigOptions->linkOrientationsX.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsY.begin(),
                             robotConfigOptions->linkOrientationsY.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsZ.begin(),
                             robotConfigOptions->linkOrientationsZ.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesX.begin(),
                             robotConfigOptions->linkLinearVelocitiesX.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesY.begin(),
                             robotConfigOptions->linkLinearVelocitiesY.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesZ.begin(),
                             robotConfigOptions->linkLinearVelocitiesZ.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesLinear.begin(),
                             robotConfigOptions->linkVelocitiesLinear.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesAngular.begin(),
                             robotConfigOptions->linkVelocitiesAngular.end());

    if (jointsInConfigFile.empty() == false and cfgPath.empty()) {
        std::string msg = "You have specified joints as part of your state variables (e.g. jointPositions or jointVelocities), ";
        msg += "but you haven't specified the 'planningEnvironmentPath' and 'executionEnvironmentPath' options which would contain the joints.";
        ERROR(msg);  
    }

    if (linksInConfigFile.empty() == false and cfgPath.empty()) {
        std::string msg = "You have specified robot links as part of your state variables (e.g. linkPoses), ";
        msg += "but you haven't specified the 'planningEnvironmentPath' and 'executionEnvironmentPath' options which would contain the robot links.";
        ERROR(msg);  
    }

    if (gazeboInterface) {
        for (auto & joint : jointsInConfigFile) {
            if (!contains(robotJointsInWorld, joint))
                ERROR("joint '" + joint + "' not defined in your robot model");
        }

        for (auto & link : linksInConfigFile) {
            if (!contains(robotLinksInWorld, link))
                ERROR("link '" + link + "' not defined in your robot model");
        }
    }

    StateSpaceInformationPtr stateSpaceInformation(new StateSpaceInformation());
    stateSpaceInformation->jointPositions = robotConfigOptions->jointPositions;
    stateSpaceInformation->jointVelocities = robotConfigOptions->jointVelocities;
    stateSpaceInformation->containedLinkPoses = robotConfigOptions->linkPoses;
    stateSpaceInformation->containedLinkPosesLowerLimits = robotConfigOptions->linkPosesLowerLimits;
    stateSpaceInformation->containedLinkPosesUpperLimits = robotConfigOptions->linkPosesUpperLimits;

    stateSpaceInformation->containedLinkPositionsX = robotConfigOptions->linkPositionsX;
    stateSpaceInformation->containedLinkPositionsXLimits = robotConfigOptions->linkPositionsXLimits;
    stateSpaceInformation->containedLinkPositionsY = robotConfigOptions->linkPositionsY;
    stateSpaceInformation->containedLinkPositionsYLimits = robotConfigOptions->linkPositionsYLimits;
    stateSpaceInformation->containedLinkPositionsZ = robotConfigOptions->linkPositionsZ;
    stateSpaceInformation->containedLinkPositionsZLimits = robotConfigOptions->linkPositionsZLimits;

    stateSpaceInformation->containedLinkOrientationsX = robotConfigOptions->linkOrientationsX;
    stateSpaceInformation->containedLinkOrientationsXLimits = robotConfigOptions->linkOrientationsXLimits;
    stateSpaceInformation->containedLinkOrientationsY = robotConfigOptions->linkOrientationsY;
    stateSpaceInformation->containedLinkOrientationsYLimits = robotConfigOptions->linkOrientationsYLimits;
    stateSpaceInformation->containedLinkOrientationsZ = robotConfigOptions->linkOrientationsZ;
    stateSpaceInformation->containedLinkOrientationsZLimits = robotConfigOptions->linkOrientationsZLimits;

    stateSpaceInformation->containedLinkLinearVelocitiesX = robotConfigOptions->linkLinearVelocitiesX;
    stateSpaceInformation->containedLinkLinearVelocitiesY = robotConfigOptions->linkLinearVelocitiesY;
    stateSpaceInformation->containedLinkLinearVelocitiesZ = robotConfigOptions->linkLinearVelocitiesZ;
    stateSpaceInformation->containedLinkLinearVelocitiesXLimits = robotConfigOptions->linkLinearVelocitiesXLimits;
    stateSpaceInformation->containedLinkLinearVelocitiesYLimits = robotConfigOptions->linkLinearVelocitiesYLimits;
    stateSpaceInformation->containedLinkLinearVelocitiesZLimits = robotConfigOptions->linkLinearVelocitiesZLimits;
    stateSpaceInformation->containedLinkVelocitiesLinear = robotConfigOptions->linkVelocitiesLinear;
    stateSpaceInformation->containedLinkVelocitiesLinearLimits = robotConfigOptions->linkVelocitiesLinearLimits;

    stateSpaceInformation->containedLinkAngularVelocitiesX = robotConfigOptions->linkAngularVelocitiesX;
    stateSpaceInformation->containedLinkAngularVelocitiesY = robotConfigOptions->linkAngularVelocitiesY;
    stateSpaceInformation->containedLinkAngularVelocitiesZ = robotConfigOptions->linkAngularVelocitiesZ;
    stateSpaceInformation->containedLinkAngularVelocitiesXLimits = robotConfigOptions->linkAngularVelocitiesXLimits;
    stateSpaceInformation->containedLinkAngularVelocitiesYLimits = robotConfigOptions->linkAngularVelocitiesYLimits;
    stateSpaceInformation->containedLinkAngularVelocitiesZLimits = robotConfigOptions->linkAngularVelocitiesZLimits;

    stateSpaceInformation->containedLinkVelocitiesAngular = robotConfigOptions->linkVelocitiesAngular;
    stateSpaceInformation->containedLinkVelocitiesAngularLimits = robotConfigOptions->linkVelocitiesAngularLimits;

    stateSpaceInformation->additionalDimensions = robotConfigOptions->additionalStateDimensions;
    stateSpaceInformation->lowerUpperLimitsAdditional = robotConfigOptions->lowerUpperLimitsAdditional;
    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointPositions;
    VectorString resolvedRedundantJointPositions;
    VectorString nonRedundantJointVelocities;
    bool breaking;

    for (size_t i = 0; i < stateSpaceInformation->jointPositions.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, stateSpaceInformation->jointPositions[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointPositions, stateSpaceInformation->jointPositions[i])) {
                nonRedundantJointPositions.push_back(stateSpaceInformation->jointPositions[i]);
            }
        }
    }

    for (size_t i = 0; i < stateSpaceInformation->jointVelocities.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, stateSpaceInformation->jointVelocities[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, stateSpaceInformation->jointVelocities[i])) {
                nonRedundantJointVelocities.push_back(stateSpaceInformation->jointVelocities[i]);
            }
        }
    }

    stateSpaceInformation->jointPositions = nonRedundantJointPositions;
    stateSpaceInformation->jointVelocities = nonRedundantJointVelocities;
    stateSpaceInformation->redundantJoints = redundantJoints;

    // determine the ordering of the state variables
    VectorString containedVariables;
    if (!(stateSpaceInformation->jointPositions.empty())) {
        containedVariables.push_back("jointPositions");
    }
    if (!(stateSpaceInformation->jointVelocities.empty())) {
        containedVariables.push_back("jointVelocities");
    }
    if (!(stateSpaceInformation->containedLinkPoses.empty())) {
        containedVariables.push_back("linkPoses");
    }
    if (!(stateSpaceInformation->containedLinkPositionsX.empty())) {
        containedVariables.push_back("linkPositionsX");
    }
    if (!(stateSpaceInformation->containedLinkPositionsY.empty())) {
        containedVariables.push_back("linkPositionsY");
    }
    if (!(stateSpaceInformation->containedLinkPositionsZ.empty())) {
        containedVariables.push_back("linkPositionsZ");
    }
    if (!(stateSpaceInformation->containedLinkOrientationsX.empty())) {
        containedVariables.push_back("linkOrientationsX");
    }
    if (!(stateSpaceInformation->containedLinkOrientationsY.empty())) {
        containedVariables.push_back("linkOrientationsY");
    }
    if (!(stateSpaceInformation->containedLinkOrientationsZ.empty())) {
        containedVariables.push_back("linkOrientationsZ");
    }
    if (!(stateSpaceInformation->containedLinkLinearVelocitiesX.empty())) {
        containedVariables.push_back("linkVelocitiesLinearX");
    }
    if (!(stateSpaceInformation->containedLinkLinearVelocitiesY.empty())) {
        containedVariables.push_back("linkVelocitiesLinearY");
    }
    if (!(stateSpaceInformation->containedLinkLinearVelocitiesZ.empty())) {
        containedVariables.push_back("linkVelocitiesLinearZ");
    }
    if (!(stateSpaceInformation->containedLinkAngularVelocitiesX.empty())) {
        containedVariables.push_back("linkVelocitiesAngularX");
    }
    if (!(stateSpaceInformation->containedLinkAngularVelocitiesY.empty())) {
        containedVariables.push_back("linkVelocitiesAngularY");
    }
    if (!(stateSpaceInformation->containedLinkAngularVelocitiesZ.empty())) {
        containedVariables.push_back("linkVelocitiesAngularZ");
    }

    SpaceVariablesPtr orderedVariables(new VectorSpaceVariables);
    std::ifstream os;
    os.open(cfgPath);
    std::string line;
    bool actOrdering = false;

    size_t startIdx = 0;

    while (std::getline(os, line)) {
        if (line.find("[state]") != std::string::npos) {
            while (!(containedVariables.empty())) {
                std::getline(os, line);
                if (line.find("#") != std::string::npos)
                    continue;
                if (line.find("jointPositions") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_POSITIONS);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->jointPositions.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointPositions");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("jointVelocities") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_VELOCITIES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->jointVelocities.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointVelocities");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPoses") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 6 * stateSpaceInformation->containedLinkPoses.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPoses");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsX") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkPositionsX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsY") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkPositionsY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsZ") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkPositionsZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsX") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkOrientationsX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsY") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkOrientationsY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsZ") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkOrientationsZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinear") != std::string::npos &&
                           !(line.find("X") != std::string::npos || line.find("Y") != std::string::npos || line.find("Z") != std::string::npos)) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 3 * stateSpaceInformation->containedLinkVelocitiesLinear.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinear");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngular") != std::string::npos &&
                           !(line.find("X") != std::string::npos || line.find("Y") != std::string::npos || line.find("Z") != std::string::npos)) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 3 * stateSpaceInformation->containedLinkVelocitiesAngular.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngular");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearX") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkLinearVelocitiesX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearY") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkLinearVelocitiesY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearZ") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkLinearVelocitiesZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularX") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkAngularVelocitiesX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularY") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkAngularVelocitiesY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularZ") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += stateSpaceInformation->containedLinkAngularVelocitiesZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                }
            }

            break;
        }
    }

    stateSpaceInformation->orderedVariables = std::move(orderedVariables);

    checkSpaceLimits(stateSpaceInformation.get(), robotConfigOptions, "stateSpace");
    return stateSpaceInformation;
}

void checkSpaceLimits(SpaceInformation* spaceInformation,
                      const RobotConfigOptions* robotConfigOptions,
                      const std::string spaceType)
{
    if (robotConfigOptions->normalizedSpaces) {
        if (spaceInformation->containedLinkPoses.size() != spaceInformation->containedLinkPosesLowerLimits.size())
            ERROR("When you use normalized spaces you need to set linkPosesLowerLimits correctly");
        if (spaceInformation->containedLinkPoses.size() != spaceInformation->containedLinkPosesUpperLimits.size())
            ERROR("When you use normalized spaces you need to set linkPosesUpperLimits correctly");

        if (spaceInformation->containedLinkPositionsX.size() != spaceInformation->containedLinkPositionsXLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionXLimits correctly");
        if (spaceInformation->containedLinkPositionsY.size() != spaceInformation->containedLinkPositionsYLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionYLimits correctly");
        if (spaceInformation->containedLinkPositionsZ.size() != spaceInformation->containedLinkPositionsZLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionZLimits correctly");

        if (spaceInformation->containedLinkOrientationsX.size() != spaceInformation->containedLinkOrientationsXLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsXLimits correctly");
        if (spaceInformation->containedLinkOrientationsY.size() != spaceInformation->containedLinkOrientationsYLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsYLimits correctly");
        if (spaceInformation->containedLinkOrientationsZ.size() != spaceInformation->containedLinkOrientationsZLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsZLimits correctly");

        if (spaceInformation->containedLinkLinearVelocitiesX.size() !=
                spaceInformation->containedLinkLinearVelocitiesXLimits.size()) {
            cout << "1: " << spaceInformation->containedLinkLinearVelocitiesX.size() << endl;
            cout << "2: " << spaceInformation->containedLinkLinearVelocitiesXLimits.size() << endl;
            ERROR("When you use normalized spaces you need to set linkVelocitiesLinearXLimits correctly");
        }
        if (spaceInformation->containedLinkLinearVelocitiesY.size() !=
                spaceInformation->containedLinkLinearVelocitiesYLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesLinearZLimits correctly");
        if (spaceInformation->containedLinkLinearVelocitiesZ.size() !=
                spaceInformation->containedLinkLinearVelocitiesZLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesLinearYLimits correctly");

        if (spaceInformation->containedLinkAngularVelocitiesX.size() !=
                spaceInformation->containedLinkAngularVelocitiesXLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesAngularXLimits correctly");
        if (spaceInformation->containedLinkAngularVelocitiesY.size() !=
                spaceInformation->containedLinkAngularVelocitiesYLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesAngularYLimits correctly");
        if (spaceInformation->containedLinkAngularVelocitiesZ.size() !=
                spaceInformation->containedLinkAngularVelocitiesZLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesAngularZLimits correctly");


        if (spaceInformation->containedLinkVelocitiesLinear.size() !=
                spaceInformation->containedLinkVelocitiesLinearLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesLinearLimits correctly");
        if (spaceInformation->containedLinkVelocitiesAngular.size() !=
                spaceInformation->containedLinkVelocitiesAngularLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesAngularLimits correctly");

        if (spaceInformation->additionalDimensions != spaceInformation->lowerUpperLimitsAdditional.size())
            ERROR("When you use normalized spaces you need to set additionalDimensionLimits correctly");
    }

    FloatType infLimit = std::numeric_limits<FloatType>::infinity();

    // ######################################################################
    // Link poses
    // ######################################################################
    if (spaceInformation->containedLinkPosesLowerLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPoses.size(); ++i) {
            VectorFloat lowerLimits;
            for (size_t j = 0; j != 6; ++j) {
                lowerLimits.push_back(-infLimit);
            }
            spaceInformation->containedLinkPosesLowerLimits.push_back(lowerLimits);
        }
    }
    if (spaceInformation->containedLinkPosesUpperLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPoses.size(); ++i) {
            VectorFloat upperLimits;
            for (size_t j = 0; j != 6; ++j) {
                upperLimits.push_back(infLimit);
            }
            spaceInformation->containedLinkPosesUpperLimits.push_back(upperLimits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPosesLowerLimits.size(); ++i) {
        if (spaceInformation->containedLinkPosesLowerLimits[i].size() != 6)
            ERROR("Wrong formatting for linkPosesLowerLimits");
        if (spaceInformation->containedLinkPosesUpperLimits[i].size() != 6)
            ERROR("Wrong formatting for linkPosesUpperLimits");
        for (size_t j = 0; j != 6; ++j) {
            if (spaceInformation->containedLinkPosesLowerLimits[i][j] >
                    spaceInformation->containedLinkPosesUpperLimits[i][j])
                ERROR("linkPosesLowerLimits must be smaller than linkPosesUpperLimits");
        }
    }

    // ######################################################################
    // Link positions
    // ######################################################################
    if (spaceInformation->containedLinkPositionsXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsXLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsXLimits[i][0] > spaceInformation->containedLinkPositionsXLimits[i][1])
            ERROR("linkPositionsXLimits is malformed");
    }
    if (spaceInformation->containedLinkPositionsYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsYLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsYLimits[i][0] > spaceInformation->containedLinkPositionsYLimits[i][1])
            ERROR("linkPositionsYLimits is malformed");
    }
    if (spaceInformation->containedLinkPositionsZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsZLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsZLimits[i][0] > spaceInformation->containedLinkPositionsZLimits[i][1])
            ERROR("linkPositionsZLimits is malformed");
    }

    // ######################################################################
    // Link orientations
    // ######################################################################
    if (spaceInformation->containedLinkOrientationsXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsXLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsXLimits[i][0] > spaceInformation->containedLinkOrientationsXLimits[i][1])
            ERROR("linkOrientationsXLimits is malformed");
    }
    if (spaceInformation->containedLinkOrientationsYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsYLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsYLimits[i][0] > spaceInformation->containedLinkOrientationsYLimits[i][1])
            ERROR("linkOrientationsYLimits is malformed");
    }
    if (spaceInformation->containedLinkOrientationsZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsZLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsZLimits[i][0] > spaceInformation->containedLinkOrientationsZLimits[i][1])
            ERROR("linkOrientationsZLimits is malformed");
    }

    // ######################################################################
    // Link velocities linear
    // ######################################################################
    if (spaceInformation->containedLinkLinearVelocitiesXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesXLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesXLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesXLimits[i][1])
            ERROR("linkLinearVelocitiesXLimits is malformed");
    }
    if (spaceInformation->containedLinkLinearVelocitiesYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesYLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesYLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesYLimits[i][1])
            ERROR("linkLinearVelocitiesYLimits is malformed");
    }
    if (spaceInformation->containedLinkLinearVelocitiesZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesZLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesZLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesZLimits[i][1])
            ERROR("linkLinearVelocitiesZLimits is malformed");
    }

    if (spaceInformation->containedLinkVelocitiesLinearLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesLinear.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkVelocitiesLinearLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesLinearLimits.size(); ++i) {
        if (spaceInformation->containedLinkVelocitiesLinearLimits[i][0] >
                spaceInformation->containedLinkVelocitiesLinearLimits[i][1])
            ERROR("linkVelocitiesLinearLimits is malformed");
    }

    // ######################################################################
    // Link velocities angular
    // ######################################################################
    if (spaceInformation->containedLinkAngularVelocitiesXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesXLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesXLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesXLimits[i][1])
            ERROR("linkAngularVelocitiesXLimits is malformed");
    }
    if (spaceInformation->containedLinkAngularVelocitiesYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesYLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesYLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesYLimits[i][1])
            ERROR("linkAngularVelocitiesYLimits is malformed");
    }
    if (spaceInformation->containedLinkAngularVelocitiesZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesZLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesZLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesZLimits[i][1])
            ERROR("linkAngularVelocitiesZLimits is malformed");
    }



    if (spaceInformation->containedLinkVelocitiesAngularLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesAngular.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkVelocitiesAngularLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesAngularLimits.size(); ++i) {
        if (spaceInformation->containedLinkVelocitiesAngularLimits[i][0] >
                spaceInformation->containedLinkVelocitiesAngularLimits[i][1])
            ERROR("linkVelocitiesAngularLimits is malformed");
    }

    if (spaceInformation->lowerUpperLimitsAdditional.size() == 0) {
        for (size_t i = 0; i != spaceInformation->additionalDimensions; ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->lowerUpperLimitsAdditional.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->lowerUpperLimitsAdditional.size(); ++i) {
        if ((spaceInformation->lowerUpperLimitsAdditional[i][0] > spaceInformation->lowerUpperLimitsAdditional[i][1]) ||
                (spaceInformation->lowerUpperLimitsAdditional[i].size() != 2))
            ERROR("additionalDimensionLimits is malformed");
    }
}

const ActionSpaceInformationPtr makeActionSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    std::string cfgPath = robotConfigOptions->configFilePath;
    VectorString robotJointsInWorld;
    if (gazeboInterface)
        robotJointsInWorld = gazeboInterface->getRobotJointNames();
    VectorString jointsInConfigFile = robotConfigOptions->jointTorques;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointPositionsAct.begin(),
                              robotConfigOptions->jointPositionsAct.end());
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointPositionsIncrementAct.begin(),
                              robotConfigOptions->jointPositionsIncrementAct.end());
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointVelocitiesAct.begin(),
                              robotConfigOptions->jointVelocitiesAct.end());

    if (jointsInConfigFile.empty() == false and cfgPath.empty()) {
        std::string msg = "You have specified joints as part of your action variables (e.g. jointTorques), ";
        msg += "but you haven't specified the 'planningEnvironmentPath' and 'executionEnvironmentPath' options which would contain the joints.";
        ERROR(msg);  
    }

    if (gazeboInterface) {
        for (auto & joint : jointsInConfigFile) {
            if (!contains(robotJointsInWorld, joint))
                ERROR("joint '" + joint + "' not defined in your robot model");
        }
    }


    ActionSpaceInformationPtr actionSpaceInformation(new ActionSpaceInformation());
    actionSpaceInformation->torqueControlledJoints = robotConfigOptions->jointTorques;
    actionSpaceInformation->velocityControlledJoints = robotConfigOptions->jointVelocitiesAct;
    actionSpaceInformation->positionControlledJoints = robotConfigOptions->jointPositionsAct;
    actionSpaceInformation->positionIncrementControlledJoints = robotConfigOptions->jointPositionsIncrementAct;
    actionSpaceInformation->additionalDimensions = robotConfigOptions->additionalActionDimensions;

    actionSpaceInformation->requiresPhysics = true;
    if (actionSpaceInformation->torqueControlledJoints.empty() &&
            actionSpaceInformation->velocityControlledJoints.empty() &&
            actionSpaceInformation->additionalDimensions == 0)
        actionSpaceInformation->requiresPhysics = false;


    for (auto & lowerUpper : robotConfigOptions->lowerUpperActionLimitsAdditional) {
        if (lowerUpper.size() != 2)
            ERROR("Lower and upper limits must be of size 2");
        std::pair<FloatType, FloatType> lu = {lowerUpper[0], lowerUpper[1]};
        actionSpaceInformation->lowerUpperLimitsAdditional.push_back(lu);
    }

    if (actionSpaceInformation->lowerUpperLimitsAdditional.size() != actionSpaceInformation->additionalDimensions)
        ERROR("Size of limits for additional action dimensions doesn't concede with number of additional dimensions");

    for (auto &lowerUpper : robotConfigOptions->jointPositionIncrementLimits) {
        if (lowerUpper.size() != 2)
            ERROR("Lower and upper limits must be of size 2");
        std::pair<FloatType, FloatType> lu = {lowerUpper[0], lowerUpper[1]};
        actionSpaceInformation->lowerUpperLimitsJointPositionIncrements.push_back(lu);
    }

    if (actionSpaceInformation->lowerUpperLimitsJointPositionIncrements.size() != actionSpaceInformation->positionIncrementControlledJoints.size()) {
        cout << "s1: " << actionSpaceInformation->lowerUpperLimitsJointPositionIncrements.size() << endl;
        cout << "s2: " << actionSpaceInformation->positionIncrementControlledJoints.size() << endl;
        ERROR("Size of limits for position increments doesn't concede with number of position increment controlled joints");
    }

    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointTorques;
    VectorString nonRedundantJointVelocities;
    VectorString nonRedundantJointPositions;
    VectorString nonRedundantJointPositionsIncrement;
    bool breaking;

    for (size_t i = 0; i < actionSpaceInformation->torqueControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->torqueControlledJoints[i])) {
                if (!contains(nonRedundantJointTorques, redundantJointList[0])) {
                    nonRedundantJointTorques.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointTorques, actionSpaceInformation->torqueControlledJoints[i])) {
                nonRedundantJointTorques.push_back(actionSpaceInformation->torqueControlledJoints[i]);
            }
        }
    }

    for (size_t i = 0; i < actionSpaceInformation->velocityControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->velocityControlledJoints[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, actionSpaceInformation->velocityControlledJoints[i])) {
                nonRedundantJointVelocities.push_back(actionSpaceInformation->velocityControlledJoints[i]);
            }
        }
    }

    for (size_t i = 0; i < actionSpaceInformation->positionControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->positionControlledJoints[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }

        if (!containing) {
            if (!contains(nonRedundantJointPositions, actionSpaceInformation->positionControlledJoints[i])) {
                nonRedundantJointPositions.push_back(actionSpaceInformation->positionControlledJoints[i]);
            }
        }
    }

    for (size_t i = 0; i != actionSpaceInformation->positionIncrementControlledJoints.size(); ++i) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->positionIncrementControlledJoints[i])) {
                if (!contains(nonRedundantJointPositionsIncrement, redundantJointList[0])) {
                    nonRedundantJointPositionsIncrement.push_back(redundantJointList[0]);
                }
            }
        }

        if (!containing) {
            if (!contains(nonRedundantJointPositionsIncrement, actionSpaceInformation->positionIncrementControlledJoints[i])) {
                nonRedundantJointPositionsIncrement.push_back(actionSpaceInformation->positionIncrementControlledJoints[i]);
            }
        }
    }

    actionSpaceInformation->torqueControlledJoints = nonRedundantJointTorques;
    actionSpaceInformation->velocityControlledJoints = nonRedundantJointVelocities;
    actionSpaceInformation->positionControlledJoints = nonRedundantJointPositions;
    actionSpaceInformation->positionIncrementControlledJoints = nonRedundantJointPositionsIncrement;
    actionSpaceInformation->redundantJoints = redundantJoints;

    // determine the ordering of the action variables
    VectorString containedVariables;
    if (!(actionSpaceInformation->torqueControlledJoints.empty())) {
        containedVariables.push_back("jointTorques");
    }
    if (!(actionSpaceInformation->velocityControlledJoints.empty())) {
        containedVariables.push_back("jointVelocities");
    }
    if (!(actionSpaceInformation->positionControlledJoints.empty())) {
        containedVariables.push_back("jointPositions");
    }
    if (!(actionSpaceInformation->positionIncrementControlledJoints.empty())) {
        containedVariables.push_back("jointPositionsIncrement");
    }

    SpaceVariablesPtr orderedVariables(new VectorSpaceVariables);
    std::ifstream os;
    os.open(cfgPath);
    std::string line;
    bool actOrdering = false;
    size_t startIdx = 0;
    while (std::getline(os, line)) {
        if (line.find("[action]") != std::string::npos) {
            while (!(containedVariables.empty())) {
                std::getline(os, line);
                if (line.find("#") != std::string::npos)
                    continue;
                if (line.find("jointTorques") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_TORQUES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += actionSpaceInformation->torqueControlledJoints.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointTorques");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("jointVelocities") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_VELOCITIES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += actionSpaceInformation->velocityControlledJoints.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointVelocities");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("jointPositions") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_POSITIONS);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += actionSpaceInformation->positionControlledJoints.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointPositions");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("jointPositionIncrements") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_POSITIONS_INCREMENT);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += actionSpaceInformation->positionIncrementControlledJoints.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointPositionsIncrement");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                }
            }
        }
    }

    actionSpaceInformation->orderedVariables = std::move(orderedVariables);

    return actionSpaceInformation;
}

const ObservationSpaceInformationPtr makeObservationSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    std::string cfgPath = robotConfigOptions->configFilePath;
    VectorString robotJointsInWorld;
    VectorString robotLinksInWorld;
    if (gazeboInterface) {
        robotJointsInWorld = gazeboInterface->getRobotJointNames();
        robotLinksInWorld = gazeboInterface->getLinkNames();

    }

    VectorString jointsInConfigFile = robotConfigOptions->jointPositionsObs;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointVelocitiesObs.begin(),
                              robotConfigOptions->jointVelocitiesObs.end());
    VectorString linksInConfigFile = robotConfigOptions->linkPosesObs;
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPosesObs.begin(),
                             robotConfigOptions->linkPosesObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsXObs.begin(),
                             robotConfigOptions->linkPositionsXObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsYObs.begin(),
                             robotConfigOptions->linkPositionsYObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPositionsZObs.begin(),
                             robotConfigOptions->linkPositionsZObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsXObs.begin(),
                             robotConfigOptions->linkOrientationsXObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsYObs.begin(),
                             robotConfigOptions->linkOrientationsYObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkOrientationsZObs.begin(),
                             robotConfigOptions->linkOrientationsZObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesXObs.begin(),
                             robotConfigOptions->linkLinearVelocitiesXObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesYObs.begin(),
                             robotConfigOptions->linkLinearVelocitiesYObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkLinearVelocitiesZObs.begin(),
                             robotConfigOptions->linkLinearVelocitiesZObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesLinearObs.begin(),
                             robotConfigOptions->linkVelocitiesLinearObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkAngularVelocitiesXObs.begin(),
                             robotConfigOptions->linkAngularVelocitiesXObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkAngularVelocitiesYObs.begin(),
                             robotConfigOptions->linkAngularVelocitiesYObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkAngularVelocitiesZObs.begin(),
                             robotConfigOptions->linkAngularVelocitiesZObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesAngular.begin(),
                             robotConfigOptions->linkVelocitiesAngular.end());

    if (jointsInConfigFile.empty() == false and cfgPath.empty()) {
        std::string msg = "You have specified joints as part of your observation variables (e.g. jointPositions or jointVelocities), ";
        msg += "but you haven't specified the 'planningEnvironmentPath' and 'executionEnvironmentPath' options which would contain the joints.";
        ERROR(msg);  
    }

    if (linksInConfigFile.empty() == false and cfgPath.empty()) {
        std::string msg = "You have specified robot links as part of your observation variables (e.g. linkPoses), ";
        msg += "but you haven't specified the 'planningEnvironmentPath' and 'executionEnvironmentPath' options which would contain the robot links.";
        ERROR(msg);  
    }

    if (gazeboInterface) {
        for (auto & joint : jointsInConfigFile) {
            if (!contains(robotJointsInWorld, joint))
                ERROR("joint '" + joint + "' not defined in your robot model");
        }

        for (auto & link : linksInConfigFile) {
            if (!contains(robotLinksInWorld, link))
                ERROR("link '" + link + "' not defined in your robot model");
        }
    }

    ObservationSpaceInformationPtr observationSpaceInformation(new ObservationSpaceInformation());
    observationSpaceInformation->jointPositions = robotConfigOptions->jointPositionsObs;
    observationSpaceInformation->jointVelocities = robotConfigOptions->jointVelocitiesObs;
    observationSpaceInformation->containedLinkPoses = robotConfigOptions->linkPosesObs;
    observationSpaceInformation->containedLinkPosesLowerLimits = robotConfigOptions->linkPosesLowerLimitsObs;
    observationSpaceInformation->containedLinkPosesUpperLimits = robotConfigOptions->linkPosesUpperLimitsObs;

    observationSpaceInformation->containedLinkPositionsX = robotConfigOptions->linkPositionsXObs;
    observationSpaceInformation->containedLinkPositionsXLimits = robotConfigOptions->linkPositionsXLimitsObs;
    observationSpaceInformation->containedLinkPositionsY = robotConfigOptions->linkPositionsYObs;
    observationSpaceInformation->containedLinkPositionsYLimits = robotConfigOptions->linkPositionsYLimitsObs;
    observationSpaceInformation->containedLinkPositionsZ = robotConfigOptions->linkPositionsZObs;
    observationSpaceInformation->containedLinkPositionsZLimits = robotConfigOptions->linkPositionsZLimitsObs;

    observationSpaceInformation->containedLinkOrientationsX = robotConfigOptions->linkOrientationsXObs;
    observationSpaceInformation->containedLinkOrientationsXLimits = robotConfigOptions->linkOrientationsXLimitsObs;
    observationSpaceInformation->containedLinkOrientationsY = robotConfigOptions->linkOrientationsYObs;
    observationSpaceInformation->containedLinkOrientationsYLimits = robotConfigOptions->linkOrientationsYLimitsObs;
    observationSpaceInformation->containedLinkOrientationsZ = robotConfigOptions->linkOrientationsZObs;
    observationSpaceInformation->containedLinkOrientationsZLimits = robotConfigOptions->linkOrientationsZLimitsObs;

    observationSpaceInformation->containedLinkLinearVelocitiesX = robotConfigOptions->linkLinearVelocitiesXObs;
    observationSpaceInformation->containedLinkLinearVelocitiesY = robotConfigOptions->linkLinearVelocitiesYObs;
    observationSpaceInformation->containedLinkLinearVelocitiesZ = robotConfigOptions->linkLinearVelocitiesZObs;
    observationSpaceInformation->containedLinkLinearVelocitiesXLimits = robotConfigOptions->linkLinearVelocitiesXLimitsObs;
    observationSpaceInformation->containedLinkLinearVelocitiesYLimits = robotConfigOptions->linkLinearVelocitiesYLimitsObs;
    observationSpaceInformation->containedLinkLinearVelocitiesZLimits = robotConfigOptions->linkLinearVelocitiesZLimitsObs;
    observationSpaceInformation->containedLinkVelocitiesLinear = robotConfigOptions->linkVelocitiesLinearObs;
    observationSpaceInformation->containedLinkVelocitiesLinearLimits = robotConfigOptions->linkVelocitiesLinearLimitsObs;

    observationSpaceInformation->containedLinkAngularVelocitiesX = robotConfigOptions->linkAngularVelocitiesXObs;
    observationSpaceInformation->containedLinkAngularVelocitiesY = robotConfigOptions->linkAngularVelocitiesYObs;
    observationSpaceInformation->containedLinkAngularVelocitiesZ = robotConfigOptions->linkAngularVelocitiesZObs;
    observationSpaceInformation->containedLinkAngularVelocitiesXLimits = robotConfigOptions->linkAngularVelocitiesXLimitsObs;
    observationSpaceInformation->containedLinkAngularVelocitiesYLimits = robotConfigOptions->linkAngularVelocitiesYLimitsObs;
    observationSpaceInformation->containedLinkAngularVelocitiesZLimits = robotConfigOptions->linkAngularVelocitiesZLimitsObs;

    observationSpaceInformation->containedLinkVelocitiesAngular = robotConfigOptions->linkVelocitiesAngular;
    observationSpaceInformation->containedLinkVelocitiesAngularLimits = robotConfigOptions->linkVelocitiesAngularLimits;

    observationSpaceInformation->additionalDimensions = robotConfigOptions->additionalObservationDimensions;
    observationSpaceInformation->lowerUpperLimitsAdditional = robotConfigOptions->lowerUpperLimitsAdditionalObs;

    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointPositions;
    VectorString nonRedundantJointVelocities;
    bool breaking;

    for (size_t i = 0; i < observationSpaceInformation->jointPositions.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, observationSpaceInformation->jointPositions[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointPositions, observationSpaceInformation->jointPositions[i])) {
                nonRedundantJointPositions.push_back(observationSpaceInformation->jointPositions[i]);
            }
        }
    }

    for (size_t i = 0; i < observationSpaceInformation->jointVelocities.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, observationSpaceInformation->jointVelocities[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, observationSpaceInformation->jointVelocities[i])) {
                nonRedundantJointVelocities.push_back(observationSpaceInformation->jointVelocities[i]);
            }
        }
    }

    observationSpaceInformation->jointPositions = nonRedundantJointPositions;
    observationSpaceInformation->jointVelocities = nonRedundantJointVelocities;

    // determine the ordering of the action variables
    VectorString containedVariables;
    if (!(observationSpaceInformation->jointVelocities.empty())) {
        containedVariables.push_back("jointVelocities");
    }
    if (!(observationSpaceInformation->jointPositions.empty())) {
        containedVariables.push_back("jointPositions");
    }
    if (!(observationSpaceInformation->containedLinkPoses.empty())) {
        containedVariables.push_back("linkPoses");
    }
    if (!(observationSpaceInformation->containedLinkPositionsX.empty())) {
        containedVariables.push_back("linkPositionsX");
    }
    if (!(observationSpaceInformation->containedLinkPositionsY.empty())) {
        containedVariables.push_back("linkPositionsY");
    }
    if (!(observationSpaceInformation->containedLinkPositionsZ.empty())) {
        containedVariables.push_back("linkPositionsZ");
    }
    if (!(observationSpaceInformation->containedLinkOrientationsX.empty())) {
        containedVariables.push_back("linkOrientationsX");
    }
    if (!(observationSpaceInformation->containedLinkOrientationsY.empty())) {
        containedVariables.push_back("linkOrientationsY");
    }
    if (!(observationSpaceInformation->containedLinkOrientationsZ.empty())) {
        containedVariables.push_back("linkOrientationsZ");
    }
    if (!(observationSpaceInformation->containedLinkVelocitiesLinear.empty())) {
        containedVariables.push_back("linkVelocitiesLinear");
    }
    if (!(observationSpaceInformation->containedLinkVelocitiesAngular.empty())) {
        containedVariables.push_back("linkVelocitiesAngular");
    }
    if (!(observationSpaceInformation->containedLinkLinearVelocitiesX.empty())) {
        containedVariables.push_back("linkVelocitiesLinearX");
    }
    if (!(observationSpaceInformation->containedLinkLinearVelocitiesY.empty())) {
        containedVariables.push_back("linkVelocitiesLinearY");
    }
    if (!(observationSpaceInformation->containedLinkLinearVelocitiesZ.empty())) {
        containedVariables.push_back("linkVelocitiesLinearZ");
    }
    if (!(observationSpaceInformation->containedLinkAngularVelocitiesX.empty())) {
        containedVariables.push_back("linkVelocitiesAngularX");
    }
    if (!(observationSpaceInformation->containedLinkAngularVelocitiesY.empty())) {
        containedVariables.push_back("linkVelocitiesAngularY");
    }
    if (!(observationSpaceInformation->containedLinkAngularVelocitiesZ.empty())) {
        containedVariables.push_back("linkVelocitiesAngularZ");
    }

    SpaceVariablesPtr orderedVariables(new VectorSpaceVariables);
    std::ifstream os;
    os.open(cfgPath);
    std::string line;
    bool actOrdering = false;
    size_t startIdx = 0;
    while (std::getline(os, line)) {
        if (line.find("[observation]") != std::string::npos) {
            while (!(containedVariables.empty())) {
                std::getline(os, line);
                if (line.find("#") != std::string::npos)
                    continue;
                if (line.find("jointPositions") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_POSITIONS);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->jointPositions.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointPositions");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("jointVelocities") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::JOINT_VELOCITIES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->jointVelocities.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "jointVelocities");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPoses") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSES);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 6 * observationSpaceInformation->containedLinkPoses.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPoses");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsX") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkPositionsX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsY") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkPositionsY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkPositionsZ") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_POSITIONS_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkPositionsZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkPositionsZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsX") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkOrientationsX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsY") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkOrientationsY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkOrientationsZ") != std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_ORIENTATIONS_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkOrientationsZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkOrientationsX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinear") != std::string::npos &&
                           !(line.find("X") != std::string::npos || line.find("Y") != std::string::npos || line.find("Z") != std::string::npos)) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 3 * observationSpaceInformation->containedLinkVelocitiesLinear.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinear");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngular") != std::string::npos &&
                           !(line.find("X") != std::string::npos || line.find("Y") != std::string::npos || line.find("Z") != std::string::npos)) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += 3 * observationSpaceInformation->containedLinkVelocitiesAngular.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngular");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearX") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkLinearVelocitiesX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearY") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkLinearVelocitiesY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesLinearZ") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_LINEAR_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkLinearVelocitiesZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesLinearZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularX") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_X);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkAngularVelocitiesX.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularX");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularY") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_Y);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkAngularVelocitiesY.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularY");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                } else if (line.find("linkVelocitiesAngularZ") != std::string::npos && line.find("Limits") == std::string::npos) {
                    orderedVariables->spaceVariables.push_back(SpaceVariable::LINK_VELOCITIES_ANGULAR_Z);
                    static_cast<VectorSpaceVariables *>(orderedVariables.get())->indices.push_back(startIdx);
                    startIdx += observationSpaceInformation->containedLinkAngularVelocitiesZ.size();
                    auto it = std::find(containedVariables.begin(), containedVariables.end(), "linkVelocitiesAngularZ");
                    auto index = std::distance(containedVariables.begin(), it);
                    containedVariables.erase(containedVariables.begin() + index);
                }
            }
        }
    }

    checkSpaceLimits(observationSpaceInformation.get(), robotConfigOptions, "observationSpace");
    observationSpaceInformation->orderedVariables = std::move(orderedVariables);

    return observationSpaceInformation;
}

}
}
