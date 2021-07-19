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
#include "oppt/robotHeaders/Serializer.hpp"

namespace oppt
{
VectorSerializer::VectorSerializer():
    RobotObjectSerializer()
{

}

RobotStateSharedPtr VectorSerializer::loadState(std::istream& is) const
{
    std::vector<std::string> strings;
    std::string s;

    // Consume s:
    is >> s;

    if (s.find("S:") != std::string::npos) {
        // Consume empty
        is >> s;
    }

    if (s == "NULL")
        return nullptr;

    while (s.find("w:") == std::string::npos) {
        strings.push_back(s);
        is >> s;
    }

    is >> s;

    FloatType weight;
    std::istringstream(s) >> weight;

    VectorFloat stateValues;
    for (size_t i = 0; i < strings.size(); i++) {
        FloatType val;
        std::istringstream(strings[i]) >> val;
        stateValues.push_back(val);
    }

    //std::getline(is, s);
    RobotStateSharedPtr state = std::make_shared<oppt::VectorState>(stateValues);
    state->as<VectorState>()->setWeight(weight);
    return state;
}

RobotStateSharedPtr VectorSerializer::loadState(const std::string& input) const
{
    std::istringstream is(input);
    return loadState(is);
}

ActionSharedPtr VectorSerializer::loadAction(std::istream& is) const
{
    std::vector<std::string> strings;
    std::string s;
    is >> s;
    if (s == "NULL") {
        return nullptr;
    }
    while (s != "END") {
        strings.push_back(s);
        if (strings.size() > 10000)
            ERROR("Failed to load action");
        is >> s;
    }

    VectorFloat actionValues(strings.size());
    for (size_t i = 0; i < strings.size(); i++) {
        FloatType val;
        std::istringstream(strings[i]) >> val;
        actionValues[i] = val;
    }

    return std::make_shared<oppt::VectorAction>(actionValues);
}

ObservationSharedPtr VectorSerializer::loadObservation(std::istream& is) const
{
    std::vector<std::string> strings;
    std::string s;
    is >> s;
    if (s == "NULL") {
        return nullptr;
    }
    while (s != "END") {
        strings.push_back(s);
        if (strings.size() > 10000)
            ERROR("Failed to load observation");
        is >> s;
    }

    VectorFloat observationValues(strings.size());
    for (size_t i = 0; i < strings.size(); i++) {
        FloatType val;
        std::istringstream(strings[i]) >> val;
        observationValues[i] = val;
    }

    return std::make_shared<oppt::VectorObservation>(observationValues);
}

std::vector<oppt::RobotStateSharedPtr> VectorSerializer::loadGoalStatesFromFile(std::string& filename) const
{
    std::vector<oppt::RobotStateSharedPtr> goalStates;
    std::ifstream file;
    try {
        file.open(filename);
    } catch (std::ios_base::failure& e) {
        std::cerr << e.what() << '\n';
        sleep(5);
    }

    FloatType dub_val;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream sin(line);
        VectorFloat stateVec;
        while (sin >> dub_val) {
            stateVec.push_back(dub_val);
        }

        oppt::RobotStateSharedPtr state = std::make_shared<oppt::VectorState>(stateVec);
        goalStates.push_back(state);
    }

    file.clear();
    file.seekg(0, file.beg);
    file.close();
    return goalStates;
}

}
