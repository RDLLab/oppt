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
#ifndef _PROBLEM_ENVIRONMENT_OPTIONS_HPP_
#define _PROBLEM_ENVIRONMENT_OPTIONS_HPP_
#include "oppt/options/Options.hpp"
#include "oppt/opptCore/typedefs.hpp"

namespace oppt
{
/** A class defining the configuration settings for ProblemEnvironment. */
struct ProblemEnvironmentOptions : public oppt::Options {
    ProblemEnvironmentOptions() = default;
    virtual ~ProblemEnvironmentOptions() = default;

    /** @brief The seed value to use for the RNG. If set to 0 a seed
     * will be obtained from /dev/random
     */
    unsigned long seed = 0;

    /** @brief A custom state to load for RNG. */
    unsigned long rngState = 0;

    /** @brief Overwrite existing log files with the same name*/
    bool overwriteExistingLogFiles = false;

    /** @brief Log Gazebo world states (careful, logfiles can become massive)
     */
    bool enableGazeboStateLogging = false;

    /** @brief Use normalized spaces internally */
    bool normalizedSpaces = false;

    /** The discount factor of the PODMP. */
    FloatType discountFactor = 1.0;

    /** @brief Determines if states for which the robot collides with an body
     * are terminal states
     */
    bool allowCollisions = true;

    /** @brief Name of the SDF file that represents the planning
     * environment
     */
    std::string planningEnvironmentPath = "";

    /** @brief Name of the SDF file that represents the execution
     * environment
     */
    std::string executionEnvironmentPath = "";

    /** @brief Name of the model in the SDF file that corresponds to the robot*/
    std::string robotName = "";

    bool interactive = false;

    /** @brief The filename of the shared library for the reward plugin user for planning */
    std::string planningRewardPlugin = "";

    /** @brief The filename of the shared library for the reward plugin user for execution */
    std::string executionRewardPlugin = "";

    /** @brief The filename of the shared library for the terminal plugin of the
     * planning environment */
    std::string planningTerminalPlugin = "";

    /** @brief The filename of the shared library for the terminal plugin of
     * the execution environment */
    std::string executionTerminalPlugin = "";

    /** @brief The filename of the shared library for the heuristic plugin */
    std::string heuristicPlugin = "";

    /** @brief The filename of the shared library for the transition plugin
     * of the execution environment
     */
    std::string executionTransitionPlugin = "";

    /** @brief The filename of the shared library for the propagator plugin
     * of the planning environment
     */
    std::string planningTransitionPlugin = "";

    /** @brief The filename of the shared library for the observation plugin
     * of the execution environment
     */
    std::string executionObservationPlugin = "";

    /** @brief The filename of the shared library for the observation plugin
     * of the planning environment
     */
    std::string planningObservationPlugin = "";

    /** @brief The filename of the shared library for the initial belief plugin */
    std::string initialBeliefPluginExecution = "";

    /** @brief The filename of the shared library for the initial belief plugin */
    std::string initialBeliefPluginPlanning = "";

    // --------------------- Simulation settings  -----------------
    /** @brief The maximum number of steps to simulate per run. */
    long nSimulationSteps = 0;
    /** @brief The number of simulations to run. */
    long nRuns = 0;
    /** @brief The maximum time (in milliseconds) to spend on each search step. */
    FloatType stepTimeout = 1000;
    // ----------------- Simulation settings: changes  ------------
    /** @brief True iff there are pre-planned model changed during the simulation. */
    bool hasChanges = false;
    /** @brief True iff the changes are dynamic. */
    bool areDynamic = false;
    /** @brief The path to the change file (relative to baseConfigPath) */
    std::string changesPath = "";

    /** @brief Serialize the belief particles **/
    bool saveParticles = false;

    /** @brief Deactivates visualization */
    bool deactivateVisualization = false;

    /** @brief Opacity of the visualized particles. 1.0 is fully opaque, 0.0 is fully transparent*/
    FloatType particleOpacity;

    /** @brief Frame rate of the visualization */
    unsigned int frameRate = 30;    

    /** @brief Makes a parser which can parse options from config files,
     * into a ProblemEnvironmentOptions instance.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser = std::make_unique<options::OptionParser>(
                    "OPPT command line interface");
        addProblemOptions(parser.get());
        addSimulationBaseOptions(parser.get());
        addGeneralOptions(parser.get());
        addChangesOptions(parser.get());
        addPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("plugins", "planningRewardPlugin", &ProblemEnvironmentOptions::planningRewardPlugin);
        parser->addOption<std::string>("plugins", "executionRewardPlugin", &ProblemEnvironmentOptions::executionRewardPlugin);
        parser->addOption<std::string>("plugins", "planningTerminalPlugin", &ProblemEnvironmentOptions::planningTerminalPlugin);
        parser->addOption<std::string>("plugins", "executionTerminalPlugin", &ProblemEnvironmentOptions::executionTerminalPlugin);
        parser->addOptionWithDefault<std::string>("plugins", "heuristicPlugin", &ProblemEnvironmentOptions::heuristicPlugin, "");
        parser->addOption<std::string>("plugins", "executionTransitionPlugin", &ProblemEnvironmentOptions::executionTransitionPlugin);
        parser->addOption<std::string>("plugins", "planningTransitionPlugin", &ProblemEnvironmentOptions::planningTransitionPlugin);
        parser->addOption<std::string>("plugins", "executionObservationPlugin", &ProblemEnvironmentOptions::executionObservationPlugin);
        parser->addOption<std::string>("plugins", "planningObservationPlugin", &ProblemEnvironmentOptions::planningObservationPlugin);
        parser->addOption<std::string>("plugins",
                                       "executionInitialBeliefPlugin",
                                       &ProblemEnvironmentOptions::initialBeliefPluginExecution);
        parser->addOption<std::string>("plugins",
                                       "planningInitialBeliefPlugin",
                                       &ProblemEnvironmentOptions::initialBeliefPluginPlanning);
    }

    static void addSimulationBaseOptions(options::OptionParser* parser) {
        parser->addOptionWithDefault<bool>("simulation",
                                           "interactive",
                                           &ProblemEnvironmentOptions::interactive, false);
        parser->addOptionWithDefault<unsigned int>("simulation",
                "particlePlotLimit",
                &Options::particlePlotLimit,
                30);
        parser->addOptionWithDefault<FloatType>("simulation", 
            "particleOpacity", 
            &ProblemEnvironmentOptions::particleOpacity, 
            0.1);
        parser->addOptionWithDefault<unsigned int>("simulation", 
            "frameRate", 
            &ProblemEnvironmentOptions::frameRate, 
            30);
    }

    static void addProblemOptions(options::OptionParser* parser) {
        parser->addOptionWithDefault<long>("problem", "nRuns", &ProblemEnvironmentOptions::nRuns, 1);
        parser->addOptionWithDefault<long>("problem", "nSteps", &ProblemEnvironmentOptions::nSimulationSteps, 200);
        parser->addOptionWithDefault<bool>("problem", "normalizedSpaces", &ProblemEnvironmentOptions::normalizedSpaces, false);
        parser->addOption<bool>("problem", "enableGazeboStateLogging", &ProblemEnvironmentOptions::enableGazeboStateLogging);
        parser->addOptionWithDefault<std::string>("problem", "planningEnvironmentPath", &ProblemEnvironmentOptions::planningEnvironmentPath, "");
        parser->addOptionWithDefault<std::string>("problem", "executionEnvironmentPath", &ProblemEnvironmentOptions::executionEnvironmentPath, "");
        parser->addOption<std::string>("problem", "robotName", &ProblemEnvironmentOptions::robotName);
        parser->addOptionWithDefault<bool>("problem", "allowCollisions", &ProblemEnvironmentOptions::allowCollisions, false);
        parser->addOption<FloatType>("problem", "discountFactor", &ProblemEnvironmentOptions::discountFactor);
        parser->addOption<FloatType>("problem", "stepTimeout", &ProblemEnvironmentOptions::stepTimeout);
    }

    static void addChangesOptions(options::OptionParser* parser) {
        parser->addOptionWithDefault("changes", "hasChanges", &ProblemEnvironmentOptions::hasChanges, false);
        parser->addOptionWithDefault("changes", "areDynamic", &ProblemEnvironmentOptions::areDynamic, false);
        parser->addOptionWithDefault<std::string>("changes", "changesPath",
                &ProblemEnvironmentOptions::changesPath, "");
    }

    static void addGeneralOptions(options::OptionParser* parser) {
        parser->addOptionWithDefault<unsigned long>("", "seed", &ProblemEnvironmentOptions::seed, 0);
        parser->addOptionWithDefault<unsigned long>("", "state", &ProblemEnvironmentOptions::rngState, 0);
        parser->addOptionWithDefault<std::string>("", "cfg", &ProblemEnvironmentOptions::configPath,
                "default.cfg");
        parser->addOptionWithDefault("", "verbose", &Options::hasVerboseOutput, false);
        parser->addValueArg("", "cfg", &Options::configPath, "", "cfg",
                            "config file path (must be an absolute path)", "path");
        parser->addOptionWithDefault<std::string>("", "logPath", &Options::logPath, "log");
        parser->addOptionWithDefault<std::string>("",
                "logFilePostfix",
                &ProblemEnvironmentOptions::logFilePostfix,
                "");
        parser->addOptionWithDefault<bool>("",
                                           "saveParticles",
                                           &ProblemEnvironmentOptions::saveParticles,
                                           false);
        parser->addOptionWithDefault<bool>("",
                                           "overwriteExistingLogFiles",
                                           &ProblemEnvironmentOptions::overwriteExistingLogFiles,
                                           true);
        parser->addOptionWithDefault<bool>("",
                                           "deactivateVisualization",
                                           &ProblemEnvironmentOptions::deactivateVisualization,
                                           false);
    }

};

typedef std::unique_ptr<ProblemEnvironmentOptions> ProblemEnvironmentOptionsPtr;

}

#endif

