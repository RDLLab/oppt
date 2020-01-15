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
#include "ChangesParser.hpp"
#include "oppt/opptCore/resources/resources.hpp"

using std::cout;
using std::endl;

using namespace oppt;

void ChangesParser::parseChanges(const std::string& changesFile,
                                 std::unordered_map<unsigned int, VectorEnvironmentChanges>& changes)
{
    LOGGING("Parsing environment changes '" + changesFile + "'");
    std::ifstream infile(changesFile);
    std::string output;
    while (!infile.eof()) {
        unsigned int t = 0;
        infile >> output;
        if (output.find("t=") != std::string::npos) {
            VectorString tElems;
            split(output, "=", tElems);
            t = atoll(tElems[tElems.size() - 1].c_str());
            if (changes.find(t) == changes.end())
                changes[t] = VectorEnvironmentChanges();
        }

        infile >> output;
        if (output.find("add") != std::string::npos) {
            std::string sdfString = "";
            if (parseAddBody(infile, sdfString)) {		
                auto addBodyChange = std::make_shared<BodyAddedChange>(sdfString, true);
                changes.at(t).push_back(addBodyChange);
            }
        } else if (output.find("remove") != std::string::npos) {
            std::string bodyName = "";
            if (parseRemoveBody(infile, bodyName)) {
                auto removeBodyChange = std::make_shared<BodyRemovedChange>(bodyName, true);
                changes.at(t).push_back(removeBodyChange);
            }
        } else if (output.find("changePose") != std::string::npos) {
            std::string bodyName = "";
            VectorFloat poseVec;
            if (parseChangePose(infile, bodyName, poseVec)) {
                geometric::Pose pose;
                pose.position = toEigenVec(VectorFloat({poseVec[0], poseVec[1], poseVec[2]}));
                pose.orientation = math::eulerAnglesToQuaternion(poseVec[3], poseVec[4], poseVec[5]);
                auto changePoseChange = std::make_shared<BodyPoseChange>(bodyName, pose, true);
                changes.at(t).push_back(changePoseChange);
            }
        }

        std::getline(infile, output);
    }
}

bool ChangesParser::parseChangePose(std::ifstream& ifstream, std::string& bodyName, VectorFloat& poseVec)
{
    std::string output;
    ifstream >> output;
    if (output.empty()) {
        WARNING("Empty body name for change body  pose change. Ignoring");
        return false;
    }

    bodyName = output;
    poseVec = VectorFloat(6, 0);
    for (size_t i = 0; i != 6; ++i) {
        ifstream >> output;
        if (output.empty()) {
            WARNING("Malformed pose for change body pose change. Ignoring");
            return false;
        }

        poseVec[i] = atof(output.c_str());
    }

    return true;
}

bool ChangesParser::parseRemoveBody(std::ifstream& ifstream, std::string& bodyName)
{
    std::string output;
    ifstream >> output;
    if (output.empty()) {
        WARNING("Empty body name for remove body change. Ignoring");
        return false;
    }

    bodyName = output;
    return true;
}

bool ChangesParser::parseAddBody(std::ifstream& ifstream, std::string& sdfString)
{
    std::string output;
    ifstream >> output;
    if (output.empty()) {
        WARNING("Empty output. Ignoring change");
        return false;
    }

    if (output.find("<sdf") != std::string::npos) {
        // Parse an sdf string
	sdfString = output + " ";        
	while (output.find("</sdf>") == std::string::npos) {
	    ifstream >> output;
	    sdfString += output + " ";	    
	}	
	
        return true;
    }
    
    if (!resources::FileExists(output)) {
        WARNING("File '" + output + "' doesn't exist. Ignoring change");
        return false;
    }

    std::string sdfFile = resources::FindFile(output);
    std::ifstream ifs(sdfFile);
    sdfString.assign((std::istreambuf_iterator<char>(ifs)),
                     (std::istreambuf_iterator<char>()));
    cout << sdfString << endl;
    return true;
}
