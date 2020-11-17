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
#include "oppt/robotHeaders/RobotImpl/RobotImplSerializer.hpp"
#include <sdf/parser.hh>
#include "oppt/robotHeaders/StateSpace.hpp"

namespace oppt
{
RobotImplSerializer::RobotImplSerializer(const std::string& worldFile):
    VectorSerializer(),
    worldFile_(worldFile),
    world_(nullptr)
{

}

void RobotImplSerializer::setWorld(const WorldPtr& world)
{
    world_ = world;
}

void RobotImplSerializer::setStateSpace(StateSpace* stateSpace)
{
    stateSpace_ = stateSpace;
}

RobotStateSharedPtr RobotImplSerializer::loadState(const std::string& input) const
{
    std::istringstream ss(input);
    RobotStateSharedPtr state = loadState(ss);
    return state;
}

OpptUserDataSharedPtr RobotImplSerializer::loadUserData(std::istream& is) const {
    return userDataSerializer_->loadUserData(is);
}


RobotStateSharedPtr RobotImplSerializer::loadState(std::istream& is) const
{    
    RobotStateSharedPtr state = VectorSerializer::loadState(is);
    std::ifstream ifs(worldFile_);
    std::string worldSDFString((std::istreambuf_iterator<char>(ifs)),
                               (std::istreambuf_iterator<char>()));
    size_t pos = worldSDFString.find("</world>");
    worldSDFString.erase(pos, 8);
    pos = worldSDFString.find("</sdf>");
    worldSDFString.erase(pos, 6);

    std::string s;
    is >> s;    
    is >> s;    

    std::vector<RobotStateSharedPtr> subStates;
    if (s.find("SUBSTATES_BEGIN") != std::string::npos) {
        while (s.find("SUBSTATES_END") == std::string::npos) {
            is >> s;
            if (s.find("SS_BEGIN") != std::string::npos) {
                is >> s;
                VectorFloat stateValues;
                while (s.find("SS_END") == std::string::npos) {
                    FloatType val;
                    std::istringstream(s) >> val;
                    stateValues.push_back(val);
                    is >> s;
                }

                RobotStateSharedPtr subStateDenormalized(new VectorState(stateValues));
                RobotStateSharedPtr subStateNormalized =
                    stateSpace_->normalizeState(subStateDenormalized);
                subStates.push_back(subStateNormalized);
            }
        }
    }

    else if (s.find("WORLDSTATE_BEGIN") != std::string::npos) {
        unsigned int counter = 0;
        while (s.find("<state") == std::string::npos) {
            is >> s;
        }

        VectorString strings( {s + " "});
        while (s.find("</state") == std::string::npos) {
            is >> s;
            strings.push_back(s + " ");
        }

        for (auto & str : strings) {
            worldSDFString += str;
        }

        worldSDFString += "</world></sdf>";
        sdf::SDFPtr sdfModel(new sdf::SDF());
        sdf::init(sdfModel);
        sdf::readString(worldSDFString, sdfModel);
        sdf::ElementPtr rootElement = sdfModel->Root();
        sdf::ElementPtr worldElement = rootElement->GetElement("world");
        sdf::ElementPtr stateElem = worldElement->GetElement("state");
        WorldStateUniquePtr worldState = std::make_unique<gazebo::physics::WorldState>(world_);
        worldState->Load(stateElem);
        GazeboWorldStatePtr gazeboWorldState = std::make_shared<GazeboWorldState>(std::move(worldState), world_);
        state->setGazeboWorldState(gazeboWorldState);
        //std::getline(is, s);
        is >> s;        
        is >> s;        
    }

    if (s.find("USER_DATA_BEGIN") != std::string::npos) {
        std::string udString = "";
        while (s.find("USER_DATA_END") == std::string::npos) {
            is >> s;
            udString += s + " ";
        }

        std::istringstream udStream(udString);
        OpptUserDataSharedPtr userData = loadUserData(udStream);
        state->setUserData(userData);
        //if (!userData)
        //    ERROR("USER DATA IS NULL");
    }

    state->setSubStates(subStates);
    return state;
}

}
