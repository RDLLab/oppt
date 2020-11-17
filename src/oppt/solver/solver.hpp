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
/** @file solver.hpp
 * Contains an abstract definition of a POMDP solver
 */
#ifndef __OPPT_SOLVER_HPP__
#define __OPPT_SOLVER_HPP__
#include <fstream>
#include <iostream>
#include <memory>

#include "oppt/global.hpp"
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/resources/resources.hpp"
//#include "problems/shared/filesystemUtils.hpp"
#include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"
#include "oppt/plugin/Plugin.hpp"

namespace oppt
{
// Forward declaration
class ProblemEnvironment;

}

using namespace oppt;
namespace solvers
{

typedef std::function<void(RobotEnvironment *const env1, RobotEnvironment *const env2)> RegisterEnvironmentFn;
typedef std::function<bool(RobotEnvironment *const env1, RobotEnvironment *const env2)> UnregisterEnvironmentFn;
typedef std::function<void(std::unique_ptr<StateNormalizer>, const bool&)> SetStateNormalizerFn;
typedef std::function<void(std::unique_ptr<ActionNormalizer>, const bool&)> SetActionNormalizerFn;
typedef std::function<void(std::unique_ptr<ObservationNormalizer>, const bool&)> SetObservationNormalizerFn;

/**
 * An abstract class which represents a POMDP solver. When designing new POMDP solvers, the user has to inherit
 * from this class and override all pure virtual methods.
 */
class Solver
{
public:
    friend class oppt::ProblemEnvironment;
    _COPY_AND_MOVE(Solver)

    /**
     * Default constructor
     */
    Solver() = default;

    /**
     * Virtual destructor
     */
    virtual ~Solver() = default;

    /**
     * @brief Get the name of the solver
     */
    const std::string getName() const {
        return solverName_;
    }

    /**
     * @brief Performs initial setup operations
     */
    virtual void setup() = 0;

    /**
     * @brief Perform reset operations. This method is called after each simulation run
     */
    virtual bool reset() = 0;

    /**
     * @brief Improve the POMDP policy starting from the current belief.
     * @param timeout The maximum time (in milliseconds) the solver should take to improve the policy
     *
     * @return true if the policy improvement was successful
     */
    virtual bool improvePolicy(const FloatType &timeout) = 0;

    /**
     * @brief Get the next action the robot has to execute according to the calculated
     * policy.
     *
     * @return A shared pointer to the Action that is going to be executed.
     * If there's no action available, a nullptr should be returned
     */
    virtual ActionSharedPtr getNextAction() = 0;

    /**
     * @brief Update the belief, based on the action taken and observation received
     * @param action The action that was executed
     * @param observation The observation that was percieved
     * @param allowTerminalStates If true, the next belief is allowed to have terminal states
     *
     * @return true if the belief update was successful
     */
    virtual bool updateBelief(const ActionSharedPtr& action,
                              const ObservationSharedPtr& observation,
                              const bool &allowTerminalStates = false) = 0;

    /**
     * @brief Method for handling environment changes. Implementing this method is optional
     *
     * @param environmentChanges Vector of oppt::EnvironmentChange pointers. The changes are ordered in sequential order
     */
    virtual void handleEnvironmentChanges(const std::vector<EnvironmentChangeSharedPtr>& environmentChanges) {}

    /**
     * @brief This method is called after a step has been performed and allows the user to serialize additional
     * information specific to a Solver implementation. Implementing this method is optional
     *
     * @param os An output stream to the log file
     */
    virtual void serializeStep(std::ofstream& os) {}

    /**
     * @brief a method which is being called when a simulation run is finished.
     * Implementing this method is optional
     *
     * @param os The output stream to the log file
     * @param run The number of the run
     */
    virtual void runFinished(std::ofstream& os, const unsigned int &run) {}


    virtual void stepFinished(const size_t &step) {}

    /**
     * @brief Returns a vector consisting of a particle representation of a belief. This method is purely used
     * for visualization purposes and is optional
     *
     * @return A vector representation of belief particles consisting of shared pointers to RobotState objects
     */
    virtual VectorRobotStatePtr getBeliefParticles() {
        return VectorRobotStatePtr();
    }

    HeuristicPlugin *getHeuristicPlugin() const {
        if (heuristicPlugin_)
            return heuristicPlugin_.get();
        return nullptr;
    }

protected:
    /**
     * @brief Registers an environment for environment changes that happen in another environment
     *
     * When a RobotEnvironment is registered for environment changes, whenever am EnvironmentChange is
     * applied to this RobotEnvironment, is will automatically be applied to the registered RobotEnvironment
     * as well. Note that only planning environments can be registered
     *
     * @param registeredEnvironment The RobotEnvironment that has to be registered for environment changes
     * @param environment The RobotEnvironment the registeredEnvironment gets registered to. If this is a nullptr,
     * the registerdEnvironment is registered to the execution environment
     */
    void registerForEnvironmentChanges(RobotEnvironment *const registeredEnvironment,
                                       RobotEnvironment *const environment = nullptr) {
        registerForEnvironmentChangesFn_(registeredEnvironment, environment);
    }

    /**
     * @brief Un-registers an environment for environment changes that happen in another environment
     */
    bool unregisterForEnvironmentChanges(RobotEnvironment *const registeredEnvironment,
                                         RobotEnvironment *const environment = nullptr) {
        return unregisterForEnvironmentChangesFn_(registeredEnvironment, environment);
    }

    /**
     * @brief Creates a heuristic plugin from the shared library provided in pluginFile
     * @param pluginFile The filename of the shared library containting the heuristic plugin
     * @return A oppt::HeuristicPlugin::HeuristicPluginUniquePtr to the created heuristic plugin
     */
    virtual HeuristicPlugin::HeuristicPluginUniquePtr createHeuristicPlugin(const std::string &pluginFile,
            RobotEnvironment *robotEnvironment) {
        if (!resources::FileExists(pluginFile)) {
            ERROR("Heuristic plugin '" + pluginFile +
                  "' doesn't exist");
        }

        HeuristicPlugin::HeuristicPluginUniquePtr heuristicPlugin = HeuristicPlugin::Create(pluginFile, "heuristicPlugin");
        heuristicPlugin->robotEnvironment_ = robotEnvironment;
        return std::move(heuristicPlugin);        
    }

    /**
     * @brief Set a custom oppt::StateNormalizer
     */
    void setStateNormalizer(std::unique_ptr<StateNormalizer> stateNormalizer, const bool &executionEnvironment) {
        setStateNormalizerFn_(std::move(stateNormalizer), executionEnvironment);
    }

    /**
     * @brief Set a custom oppt::ActionNormalizer
     */
    void setActionNormalizer(std::unique_ptr<ActionNormalizer> actionNormalizer, const bool &executionEnvironment) {
        setActionNormalizerFn_(std::move(actionNormalizer), executionEnvironment);
    }

    /**
     * @brief Set a custom oppt::ObservationNormalizer
     */
    void setObservationNormalizer(std::unique_ptr<ObservationNormalizer> observationNormalizer, const bool &executionEnvironment) {
        setObservationNormalizerFn_(std::move(observationNormalizer), executionEnvironment);
    }

protected:
    /** @brief The planning environment the solver uses for computing a policy */
    oppt::RobotEnvironment* robotPlanningEnvironment_ = nullptr;
    /** @brief The name of the solver */
    std::string solverName_ = "Solver";
    /** @brief A std::unique_ptr to the oppt::HeuristicPlugin. If no HeuristicPlugin is provided in the problem configuration file, this will be a nullptr */
    HeuristicPlugin::HeuristicPluginUniquePtr heuristicPlugin_;
    /** @brief A pointer to the ProblemEnvironmentOptions object*/
    ProblemEnvironmentOptions* problemEnvironmentOptions_ = nullptr;
    /** @brief A pointer to the random engine*/
    RandomEnginePtr randGen_ = nullptr;

    RegisterEnvironmentFn registerForEnvironmentChangesFn_;

    UnregisterEnvironmentFn unregisterForEnvironmentChangesFn_;

    SetStateNormalizerFn setStateNormalizerFn_;

    SetActionNormalizerFn setActionNormalizerFn_;

    SetObservationNormalizerFn setObservationNormalizerFn_;

private:
    void setRobotPlanningEnvironment(oppt::RobotEnvironment* robotPlanningEnvironment) {
        robotPlanningEnvironment_ = robotPlanningEnvironment;
    }

    void setProblemEnvironmentOptions(ProblemEnvironmentOptions* problemEnvironmentOptions) {
        problemEnvironmentOptions_ = problemEnvironmentOptions;
    }

    void setRandomEngine(RandomEnginePtr& randomEngine) {
        randGen_ = randomEngine;
    }

    void setRegisterEnvironmentFn(RegisterEnvironmentFn &registerEnvFn) {
        registerForEnvironmentChangesFn_ = registerEnvFn;
    }

    void setUnregisterEnvironmentFn(UnregisterEnvironmentFn &unregisterEnvFn) {
        unregisterForEnvironmentChangesFn_ = unregisterEnvFn;
    }

    void setSetStateNormalizerFn(SetStateNormalizerFn &setStateNormalizerFn) {
        setStateNormalizerFn_ = setStateNormalizerFn;
    }

    void setSetActionNormalizerFn(SetActionNormalizerFn &setActionNormalizerFn) {
        setActionNormalizerFn_ = setActionNormalizerFn;
    }

    void setSetObservationNormalizerFn(SetObservationNormalizerFn &setObservationNormalizerFn) {
        setObservationNormalizerFn_ = setObservationNormalizerFn;
    }
};

typedef std::unique_ptr<Solver> SolverPtr;

}


#endif


