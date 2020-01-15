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
#ifndef __CHANGES_PARSER_HPP__
#define __CHANGES_PARSER_HPP__
#include "oppt/opptCore/core.hpp"
#include "oppt/robotEnvironment/include/EnvironmentChanges.hpp"

namespace oppt
{

/**
 * Parser that reads environment changes from a file and constructs a oppt::VectorEnvironmentChanges from them
 */
class ChangesParser
{
public:
    /** @brief Default constructor */
    ChangesParser() = default;

    /**
     * Parses the environment changes
     * @param changesFile The file containing the environment changes
     * @param changes An unordered_map where the changes are stored in
     */
    void parseChanges(const std::string& changesFile, std::unordered_map<unsigned int, VectorEnvironmentChanges>& changes);

    /**
     * Parses an add body change
     * @param ifstream Input stream to the changes file
     * @param sdfString SDF containting a description of the body that is being added
     */
    bool parseAddBody(std::ifstream& ifstream, std::string& sdfString);

    /**
     * Parses an remove body change
     * @param ifstream Input stream to the changes file
     * @param bodyName The name of the body that is being removed
     */
    bool parseRemoveBody(std::ifstream& ifstream, std::string& bodyName);

    /**
     * Parses an change body pose change
     * @param ifstream Input stream to the changes file
     * @param bodyName The name of the body whose pose is being changed
     * @param poseVec A 6D vector containing the new pose
     */
    bool parseChangePose(std::ifstream& ifstream, std::string& bodyName, VectorFloat& poseVec);

};
}

#endif
