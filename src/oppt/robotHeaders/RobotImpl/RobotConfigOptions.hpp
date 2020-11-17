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
#ifndef _ROBOT_CONFIG_OPTIONS_
#define _ROBOT_CONFIG_OPTIONS_
#include "oppt/options/Options.hpp"
#include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"
#include "oppt/opptCore/typedefs.hpp"

namespace oppt
{
class RobotConfigOptions: public oppt::ProblemEnvironmentOptions
{
public:
    RobotConfigOptions() = default;
    virtual ~RobotConfigOptions() = default;

    bool normalizedSpaces = false;

    /** ---------- State options ------------*/
    VectorString jointPositions;

    VectorString jointVelocities;

    VectorString linkPoses;

    std::vector<VectorFloat> linkPosesLowerLimits;

    std::vector<VectorFloat> linkPosesUpperLimits;

    VectorString linkPositionsX;

    std::vector<VectorFloat> linkPositionsXLimits;

    VectorString linkPositionsY;

    std::vector<VectorFloat> linkPositionsYLimits;

    VectorString linkPositionsZ;

    std::vector<VectorFloat> linkPositionsZLimits;

    VectorString linkOrientationsX;

    std::vector<VectorFloat> linkOrientationsXLimits;

    VectorString linkOrientationsY;

    std::vector<VectorFloat> linkOrientationsYLimits;

    VectorString linkOrientationsZ;

    std::vector<VectorFloat> linkOrientationsZLimits;

    VectorString linkLinearVelocitiesX;

    std::vector<VectorFloat> linkLinearVelocitiesXLimits;

    VectorString linkLinearVelocitiesY;

    std::vector<VectorFloat> linkLinearVelocitiesYLimits;

    VectorString linkLinearVelocitiesZ;

    std::vector<VectorFloat> linkLinearVelocitiesZLimits;

    VectorString linkAngularVelocitiesX;

    std::vector<VectorFloat> linkAngularVelocitiesXLimits;

    VectorString linkAngularVelocitiesY;

    std::vector<VectorFloat> linkAngularVelocitiesYLimits;

    VectorString linkAngularVelocitiesZ;

    std::vector<VectorFloat> linkAngularVelocitiesZLimits;

    VectorString linkVelocitiesLinear;

    std::vector<VectorFloat> linkVelocitiesLinearLimits;

    VectorString linkVelocitiesAngular;

    std::vector<VectorFloat> linkVelocitiesAngularLimits;

    unsigned int additionalStateDimensions = 0;

    std::vector<VectorFloat> lowerUpperLimitsAdditional;

    /** ---------- Action options ------------*/
    VectorString jointTorques;

    VectorString jointPositionsAct;

    VectorString jointPositionsIncrementAct;

    VectorString jointVelocitiesAct;

    unsigned int additionalActionDimensions = 0;

    std::vector<VectorFloat> lowerUpperActionLimitsAdditional;

    std::vector<VectorFloat> jointPositionIncrementLimits;

    /** ---------- Observation options ------------*/
    VectorString jointPositionsObs;

    VectorString jointVelocitiesObs;

    VectorString linkPosesObs;

    std::vector<VectorFloat> linkPosesLowerLimitsObs;

    std::vector<VectorFloat> linkPosesUpperLimitsObs;

    VectorString linkPositionsXObs;

    std::vector<VectorFloat> linkPositionsXLimitsObs;

    VectorString linkPositionsYObs;

    std::vector<VectorFloat> linkPositionsYLimitsObs;

    VectorString linkPositionsZObs;

    std::vector<VectorFloat> linkPositionsZLimitsObs;

    VectorString linkOrientationsXObs;

    std::vector<VectorFloat> linkOrientationsXLimitsObs;

    VectorString linkOrientationsYObs;

    std::vector<VectorFloat> linkOrientationsYLimitsObs;

    VectorString linkOrientationsZObs;

    std::vector<VectorFloat> linkOrientationsZLimitsObs;

    VectorString linkLinearVelocitiesXObs;

    std::vector<VectorFloat> linkLinearVelocitiesXLimitsObs;

    VectorString linkLinearVelocitiesYObs;

    std::vector<VectorFloat> linkLinearVelocitiesYLimitsObs;

    VectorString linkLinearVelocitiesZObs;

    std::vector<VectorFloat> linkLinearVelocitiesZLimitsObs;

    VectorString linkAngularVelocitiesXObs;

    std::vector<VectorFloat> linkAngularVelocitiesXLimitsObs;

    VectorString linkAngularVelocitiesYObs;

    std::vector<VectorFloat> linkAngularVelocitiesYLimitsObs;

    VectorString linkAngularVelocitiesZObs;

    std::vector<VectorFloat> linkAngularVelocitiesZLimitsObs;

    VectorString linkVelocitiesLinearObs;

    std::vector<VectorFloat> linkVelocitiesLinearLimitsObs;

    VectorString linkVelocitiesAngularObs;

    std::vector<VectorFloat> linkVelocitiesAngularLimitsObs;

    unsigned int additionalObservationDimensions = 0;

    std::vector<VectorFloat> lowerUpperLimitsAdditionalObs;

    bool ignoreGazeboSensors = false;

    std::string configFilePath = "";

    /** ---------- Options options ------------*/
    std::vector<VectorString> redundantJoints;

    VectorString collisionInvariantLinks;

    static std::unique_ptr<options::OptionParser> makeParser(const std::string &configPath) {
        std::unique_ptr<options::OptionParser> parser = std::make_unique<options::OptionParser>(
                    "OPPT command line interface");
        addStateOptions(parser.get());
        addActionOptions(parser.get());
        addObservationOptions(parser.get());
        addOptionsOptions(parser.get());
        //configFilePath = configPath;
        parser->addOptionWithDefault<std::string>("", "configFilePath", &RobotConfigOptions::configFilePath, configPath);
        return std::move(parser);
    }

    static void addStateOptions(options::OptionParser* parser) {
        VectorString defaultStringVec;
        std::vector<VectorFloat> defaultLimitsVec;
        parser->addOptionWithDefault<VectorString>("state", "jointPositions", &RobotConfigOptions::jointPositions, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "jointVelocities", &RobotConfigOptions::jointVelocities, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkPoses", &RobotConfigOptions::linkPoses, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",


                "linkPosesLowerLimits",
                &RobotConfigOptions::linkPosesLowerLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkPosesUpperLimits",
                &RobotConfigOptions::linkPosesUpperLimits,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("state", "linkPositionsX", &RobotConfigOptions::linkPositionsX, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkPositionsY", &RobotConfigOptions::linkPositionsY, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkPositionsZ", &RobotConfigOptions::linkPositionsZ, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkPositionXLimits",
                &RobotConfigOptions::linkPositionsXLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkPositionYLimits",
                &RobotConfigOptions::linkPositionsYLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkPositionZLimits",
                &RobotConfigOptions::linkPositionsZLimits,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("state", "linkOrientationsX", &RobotConfigOptions::linkOrientationsX, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkOrientationsY", &RobotConfigOptions::linkOrientationsY, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkOrientationsZ", &RobotConfigOptions::linkOrientationsZ, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkOrientationsXLimits",
                &RobotConfigOptions::linkOrientationsXLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkOrientationsYLimits",
                &RobotConfigOptions::linkOrientationsYLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkOrientationsZLimits",
                &RobotConfigOptions::linkOrientationsZLimits,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesLinearX", &RobotConfigOptions::linkLinearVelocitiesX, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesLinearY", &RobotConfigOptions::linkLinearVelocitiesY, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesLinearZ", &RobotConfigOptions::linkLinearVelocitiesZ, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesLinearXLimits",
                &RobotConfigOptions::linkLinearVelocitiesXLimits, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesLinearYLimits",
                &RobotConfigOptions::linkLinearVelocitiesYLimits, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesLinearZLimits",
                &RobotConfigOptions::linkLinearVelocitiesZLimits, defaultLimitsVec);

        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesAngularX", &RobotConfigOptions::linkAngularVelocitiesX, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesAngularY", &RobotConfigOptions::linkAngularVelocitiesY, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesAngularZ", &RobotConfigOptions::linkAngularVelocitiesZ, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesAngularXLimits",
                &RobotConfigOptions::linkAngularVelocitiesXLimits, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesAngularYLimits",
                &RobotConfigOptions::linkAngularVelocitiesYLimits, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesAngularZLimits",
                &RobotConfigOptions::linkAngularVelocitiesZLimits, defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesLinear", &RobotConfigOptions::linkVelocitiesLinear, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesLinearLimits",
                &RobotConfigOptions::linkVelocitiesLinearLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<VectorString>("state", "linkVelocitiesAngular", &RobotConfigOptions::linkVelocitiesAngular, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "linkVelocitiesAngularLimits",
                &RobotConfigOptions::linkVelocitiesAngularLimits,
                defaultLimitsVec);
        parser->addOptionWithDefault<unsigned int>("state", "additionalDimensions", &RobotConfigOptions::additionalStateDimensions, 0);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("state",
                "additionalDimensionLimits",
                &RobotConfigOptions::lowerUpperLimitsAdditional,
                defaultLimitsVec);
    }

    static void addObservationOptions(options::OptionParser* parser) {
        VectorString defaultStringVec;
        std::vector<VectorFloat> defaultLimitsVec;
        parser->addOptionWithDefault<VectorString>("observation", "jointPositions", &RobotConfigOptions::jointPositionsObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "jointVelocities", &RobotConfigOptions::jointVelocitiesObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkPoses", &RobotConfigOptions::linkPosesObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkPosesLowerLimits",
                &RobotConfigOptions::linkPosesLowerLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkPosesUpperLimits",
                &RobotConfigOptions::linkPosesUpperLimitsObs,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("observation", "linkPositionsX", &RobotConfigOptions::linkPositionsXObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkPositionsY", &RobotConfigOptions::linkPositionsYObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkPositionsZ", &RobotConfigOptions::linkPositionsZObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkPositionXLimits",
                &RobotConfigOptions::linkPositionsXLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkPositionYLimits",
                &RobotConfigOptions::linkPositionsYLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkPositionZLimits",
                &RobotConfigOptions::linkPositionsZLimitsObs,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("observation", "linkOrientationsX", &RobotConfigOptions::linkOrientationsXObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkOrientationsY", &RobotConfigOptions::linkOrientationsYObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkOrientationsZ", &RobotConfigOptions::linkOrientationsZObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkOrientationsXLimits",
                &RobotConfigOptions::linkOrientationsXLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkOrientationsYLimits",
                &RobotConfigOptions::linkOrientationsYLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkOrientationsZLimits",
                &RobotConfigOptions::linkOrientationsZLimitsObs,
                defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesLinearX", &RobotConfigOptions::linkLinearVelocitiesXObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesLinearY", &RobotConfigOptions::linkLinearVelocitiesYObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesLinearZ", &RobotConfigOptions::linkLinearVelocitiesZObs, defaultStringVec);        
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesLinearXLimits",
                &RobotConfigOptions::linkLinearVelocitiesXLimitsObs, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesLinearYLimits",
                &RobotConfigOptions::linkLinearVelocitiesYLimitsObs, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesLinearZLimits",
                &RobotConfigOptions::linkLinearVelocitiesZLimitsObs, defaultLimitsVec);

        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesAngularX", &RobotConfigOptions::linkAngularVelocitiesXObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesAngularY", &RobotConfigOptions::linkAngularVelocitiesYObs, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesAngularZ", &RobotConfigOptions::linkAngularVelocitiesZObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesAngularXLimits",
                &RobotConfigOptions::linkAngularVelocitiesXLimitsObs, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesAngularYLimits",
                &RobotConfigOptions::linkAngularVelocitiesYLimitsObs, defaultLimitsVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesAngularZLimits",
                &RobotConfigOptions::linkAngularVelocitiesZLimitsObs, defaultLimitsVec);


        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesLinear", &RobotConfigOptions::linkVelocitiesLinearObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesLinearLimits",
                &RobotConfigOptions::linkVelocitiesLinearLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<VectorString>("observation", "linkVelocitiesAngular", &RobotConfigOptions::linkVelocitiesAngularObs, defaultStringVec);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "linkVelocitiesAngularLimits",
                &RobotConfigOptions::linkVelocitiesAngularLimitsObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<unsigned int>("observation", "additionalDimensions", &RobotConfigOptions::additionalObservationDimensions, 0);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("observation",
                "additionalDimensionLimits",
                &RobotConfigOptions::lowerUpperLimitsAdditionalObs,
                defaultLimitsVec);
        parser->addOptionWithDefault<bool>("observation", "ignoreGazeboSensors", &RobotConfigOptions::ignoreGazeboSensors, false);
    }

    static void addActionOptions(options::OptionParser* parser) {
        VectorString defaultStringVec;
        std::vector<VectorFloat> defaultAdditionalLimits;
        parser->addOptionWithDefault<VectorString>("action", "jointTorques", &RobotConfigOptions::jointTorques, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("action", "jointPositions", &RobotConfigOptions::jointPositionsAct, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("action", "jointPositionIncrements", &RobotConfigOptions::jointPositionsIncrementAct, defaultStringVec);
        parser->addOptionWithDefault<VectorString>("action", "jointVelocities", &RobotConfigOptions::jointVelocitiesAct, defaultStringVec);
        parser->addOptionWithDefault<unsigned int>("action",
                "additionalDimensions",
                &RobotConfigOptions::additionalActionDimensions,
                0);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("action", 
            "jointPositionIncrementLimits", 
            &RobotConfigOptions::jointPositionIncrementLimits, 
            defaultAdditionalLimits);
        parser->addOptionWithDefault<std::vector<VectorFloat>>("action",
                "additionalDimensionLimits",
                &RobotConfigOptions::lowerUpperActionLimitsAdditional,
                defaultAdditionalLimits);
    }

    static void addOptionsOptions(options::OptionParser* parser) {
        std::vector<VectorString> defaultVStringVec;
        VectorString defaultStringVec;
        parser->addOptionWithDefault<std::vector<VectorString>>("options",
                "redundantJoints",
                &RobotConfigOptions::redundantJoints,
                defaultVStringVec);
        parser->addOptionWithDefault<VectorString>("options",
                "collisionInvariantLinks",
                &RobotConfigOptions::collisionInvariantLinks,
                defaultStringVec);
        parser->addOptionWithDefault<bool>("problem", "normalizedSpaces", &RobotConfigOptions::normalizedSpaces, false);
    }

};
}

#endif
