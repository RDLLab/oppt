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
#ifndef _ROBOT_ENVIRONMENT_HPP_
#define _ROBOT_ENVIRONMENT_HPP_
#include <assert.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
//#include "BoxBody.hpp"
//#include "SphereBody.hpp"
//#include "MeshBody.hpp"
#include "SDFParser.hpp"
#include "oppt/robotHeaders/robot.hpp"
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/resources/resources.hpp"
#include "EnvironmentChanges.hpp"
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/global.hpp"

namespace oppt
{

typedef std::function<void(const EnvironmentChangeSharedPtr&)> ProblemEnvironmentChangeHandlerFn;

class RewardPlugin;
class TerminalPlugin;
class InitialBeliefPlugin;

class RobotEnvironment
{
public:
    friend class ProblemEnvironment;

    typedef std::function<void(const EnvironmentChangeSharedPtr&)> OnEnvironmentChangedFunction;

    RobotEnvironment();

    ~RobotEnvironment();

    /**
     * @brief Returns a pointer to the robot that operates within this RobotEnvironment
     */
    oppt::Robot* getRobot() const;

    /**
     * @brief Clones the RobotEnvironment
     * @return A RobotEnvironmentSharedPtr to an independend instance of the cloned environment
     */
    RobotEnvironmentSharedPtr clone(const unsigned int& uid = oppt::UID::getUniqueId());

    /**
     * @brief Get the threadID of this environment
     */
    const unsigned int getThreadId() const;

    /**
     * @brief Returns true if this environment is the execution environment
     */
    const bool isExecutionEnvironment() const;

    /**
     * @brief Adds an environment change that will be applied when calling applyChanges()
     *
     * @param environmentChange A shared pointer to the added EnvironmentChange
     */
    void addEnvironmentChange(const EnvironmentChangeSharedPtr& environmentChange);

    /**
     * @brief Apply all environment changes that have been added using the addEnvironmentChange API
     */
    void applyChanges();

    /**
     * @brief Get the environment prefix
     */
    const std::string getPrefix() const;

    /**
     * @brief Gets the filename of the environment this environment was loaded with
     */
    const std::string getWorldFile() const;

    /**
     * @brief Get a pointer to the underlying scene
     */
    const SceneSharedPtr getScene() const;

    /**
     * @brief Get the EnvironmentInfo
     */
    EnvironmentInfoSharedPtr getEnvironmentInfo() const;

    /**
     * @brief Get a reward based on a PropagationResult
     * This method uses the loaded RewardPlugin to sample a reward
     *
     * @return The sampled reward
     */
    FloatType getReward(const PropagationResultSharedPtr& propagationResult) const;

    /**
     * @brief Determines if the nextState in the PropagationResult is a terminal state
     *
     * This method uses the oppt::TerminalPlugin to determine if the nextState
     * inside the PropagationResult is a terminal state
     */
    bool isTerminal(const PropagationResultSharedPtr& propagationResult) const;

    /**
     * @brief Determines is the nextState in the PropagationResult is valid
     *
     * This method uses the oppt::TerminalPlugin to determine if the nextState
     * inside the PropagationResult is valid
     */
    bool isValid(const PropagationResultSharedPtr& propagationResult) const;

    /**
     * @brief Returns a pointer to the oppt::RewardPlugin
     */
    RewardPlugin *const getRewardPlugin() const;

    /**
     * @brief Sample an inital state
     *
     * Samples an intial RobotState from the set of user defined-inital states using uniform sampling
     *
     * @return A shared pointer to the sampled initial state
     */
    RobotStateSharedPtr sampleInitialState() const;

    /**
     * @brief Returns a pointer to the GazeboInterface instance
     */
    GazeboInterface* const getGazeboInterface() const;

    /**
     * @brief Returns a pointer to the oppt::InitialBeliefPlugin
     */
    InitialBeliefPlugin *const getInitialBeliefPlugin() const;

    /**
     * @brief Returns a pointer to the oppt::TerminalPlugin
     */
    TerminalPlugin *const getTerminalPlugin() const;

    void createInitialBeliefPlugin(const std::string& pluginFile);

    void createRewardPlugin(const std::string& pluginFile);

    void createTerminalPlugin(const std::string& pluginFile);

    void loadInitialBeliefPlugin(const std::string& optionsFile);

    void loadRewardPlugin(const std::string& optionsFile);

    void loadTerminalPlugin(const std::string& optionsFile);  

private:
    template <class RobotType>
    bool createRobot(std::string worldFile,
                     std::string robotName,
                     std::string configFile,
                     std::string prefix,
                     unsigned int threadId = 0) {
        worldFile_ = worldFile;
        prefix_ = prefix;
        robotName_ = robotName;
        configPath_ = configFile;
        robot_ = std::make_unique<RobotType>(worldFile_,
                                             robotName,
                                             configFile,
                                             prefix_,
                                             threadId);
        if (worldFile_.empty() == false) {
            if (!gazeboInterface_)
                makeGazeboInterface();
            robot_->setGazeboInterface(gazeboInterface_.get());
        }
        robot_->setRandomEngine(randomEngine_);
        return true;
    }

    bool makeGazeboInterface();

    void initializeEnvironmentCallbacks();

    void registerForEnvironmentChanges(RobotEnvironment* planningEnvironment);

    bool unregisterForEnvironmentChanges(RobotEnvironment *const planningEnvironment);

    bool loadEnvironment(std::string environment_file, const std::string& robotName, const bool& reset = false);

    void makeEnvironmentInfo(const bool& interactive);

    void setEnvironmentInfo(std::shared_ptr<oppt::EnvironmentInfo>& environmentInfo);

    bool setExecutionEnvironment(const bool& executionEnvironment);

    void setProblemEnvironmentChangeHandlerFn(ProblemEnvironmentChangeHandlerFn problemEnvironmentChangeHandlerFn);

    bool addBody(BodyUniquePtr body, bool executeCallbacks = true);

    bool removeModel(const std::string &modelName);

    bool removeBody(std::string bodyName, bool executeCallbacks = true);

    bool changeBodyPose(const std::string& name, const geometric::Pose& pose, bool executeCallbacks = true);

    void setThreadId(const unsigned int& threadId);

    void setRandomEngine(RandomEnginePtr randomEngine);

private:
    std::string robotName_;

    std::string configPath_;

    std::string prefix_;

    std::string worldFile_;

    std::unique_ptr<oppt::Robot> robot_;

    RandomEnginePtr randomEngine_;

    mutable oppt::EnvironmentInfoSharedPtr environmentInfo_;

    oppt::SceneSharedPtr scene_;

    std::vector<RobotEnvironment*> getRegisteredRobotEnvironments() const;

    std::unique_ptr<RewardPlugin> rewardPlugin_;

    std::unique_ptr<TerminalPlugin> terminalPlugin_;

    std::unique_ptr<InitialBeliefPlugin> initialBeliefPlugin_;

    std::string rewardPluginFile_ = "";

    std::string rewardPluginOptionsFile_ = "";

    std::string terminalPluginFile_ = "";

    std::string terminalPluginOptionsFile_ = "";

    std::string initialBeliefPluginFile_ = "";

    std::string initialBeliefOptionsFile_ = "";

    void notifyPlanningEnvironments(const EnvironmentChangeSharedPtr& environmentChange);

    void onAddBody(const std::string& str);

    void onRemoveBody(const std::string& str);

    void onBodyPoseChanged(const std::string& str, const geometric::Pose& pose);

    void onEnvironmentChanged(const EnvironmentChangeSharedPtr& environmentChange);

    std::function<void(const std::string&)> onAddBodyFn_;

    std::function<void(const std::string&)> onRemoveBodyFn_;

    std::function<void(const std::string&, const geometric::Pose&)> onBodyPoseChangedFn_ = NULL;

    std::vector<OnEnvironmentChangedFunction> onEnvironmentChangedFns_;

    unsigned int threadId_;

    bool isExecutionEnvironment_ = false;

    EnvironmentChangesPtr environmentChanges_;

    ProblemEnvironmentChangeHandlerFn problemEnvironmentChangeHandlerFn_ = NULL;

    std::vector<RobotEnvironment*> registeredRobotEnvironments_;

    std::unique_ptr<GazeboInterface> gazeboInterface_;
};
}

#endif
