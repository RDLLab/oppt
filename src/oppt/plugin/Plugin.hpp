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
#ifndef __OPPT_PLUGIN_HPP__
#define __OPPT_PLUGIN_HPP__
#include <sys/types.h>
#include <sys/stat.h>
#include <dlfcn.h>
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/resources/resources.hpp"
#include "oppt/robotEnvironment/include/RobotEnvironment.hpp"
#include "PluginOptions.hpp"
#include "oppt/opptCore/ObservationReport.hpp"
#include "oppt/opptCore/HeuristicInfo.hpp"
#include "oppt/opptCore/utils.hpp"
#include "oppt/opptCore/Distribution.hpp"

namespace solvers {
    class Solver;
}

namespace oppt
{

/**
 * Eunm of the plugin types
 */
enum PluginType {
    REWARD_PLUGIN,
    HEURISTIC_PLUGIN,
    TRANSITION_PLUGIN,
    OBSERVATION_PLUGIN,
    TERMINAL_PLUGIN,
    INITIAL_BELIEF_PLUGIN
};

/**
 * Base template type for every PluginType
 */
template<class T>
class OpptPlugin
{
public:
    _NO_COPY_OR_MOVE(OpptPlugin)
    typedef std::unique_ptr<T> TPtr;

    OpptPlugin() {
        this->dlHandle = NULL;
    }

    virtual ~OpptPlugin() = default;

    template<class C>
    C* as() {
        return static_cast<C*>(this);
    }

    static TPtr Create(const std::string& filename,
                       const std::string& name) {
        std::string fullFileName = "";
        if (oppt::resources::FileExists(filename)) {
            fullFileName = oppt::resources::FindFile(filename);
        } else {
            ERROR("File '" + filename + "' not found");
        }
        
        bool found = false;
        std::string registerName = "RegisterPlugin";        
        //void* dlHandle = dlopen(fullFileName.c_str(), RTLD_LAZY | RTLD_GLOBAL);
        void* dlHandle = dlopen(fullFileName.c_str(), RTLD_NOW | RTLD_GLOBAL);
        if (!dlHandle) {
            cout << dlerror() << endl;
            ERROR("Failed to load plugin: '" + fullFileName + "'");
        }

        fptr_union_t registerFunc;

        registerFunc.ptr = dlsym(dlHandle, registerName.c_str());
        if (!registerFunc.ptr) {
            ERROR("Failed to register plugin: '" + fullFileName + "'");
        }

        TPtr result;
        result.reset(registerFunc.func());
        result->dlHandle = dlHandle;
        result->handleName = name;
        result->filename = filename;
        return result;
    }

    /**
     * @brief Load method that is called after a plugin has been created
     *
     * Here the user should initialize any custom datastructures required by this plugin
     *
     * @param optionsFile The full path to the problem configuration file. Can be used to parse
     * custom options
     *
     * @return A bool indicating if loading the plugin as successful
     */
    virtual bool load(const std::string& optionsFile) = 0;

    /**
     * @brief Get the plugin Type
     *
     * @return The PluginType
     */
    PluginType getType() const {
        return type_;
    }

    /**
     * @brief Get the shared library filename this plugin was loaded from
     */
    std::string getFilename() const {
        return filename;
    }

    /**
     * @brief Get the handle name
     */
    std::string getHandleName() const {
        return handleName;
    }

    /**
     * @brief Unloads the plugin
     */
    void close() {
        if (dlHandle)
            dlclose(dlHandle);
    }

    /**
     * @brief Set the plugin user data. This can be used to
     * share data structures amongst different plugins. Implementing
     * this method is optional
     *
     * @param userData void pointer to the user data object
     */
    virtual void setUserData(const OpptUserDataSharedPtr& userData) {
        userData_ = userData;
    }

    /**
     * @brief Get the user data.
     * Implementing this method is optional
     *
     * @return oppt::OpptUserDataSharedPtr containing the user data
     */
    virtual const OpptUserDataSharedPtr getUserData() const {
        return userData_;
    }

    /**
     * @brief Get the options used in this plugin
     *
     * @return A pointer to the oppt::PluginOptions object
     */
    PluginOptions *const getOptions() const {
        return options_.get();
    }

protected:
    PluginType type_;

    /** @brief A pointer to the oppt::RobotEnvironment this plugin belongs to */
    RobotEnvironment *robotEnvironment_ = nullptr;

    /** @brief unique_ptr to the PluginOptions object.*/
    std::unique_ptr<PluginOptions> options_;

    /** @brief Shared pointer to the oppt::OpptUserData */
    OpptUserDataSharedPtr userData_ = nullptr;

protected:
    /**
     * @brief Parses the plugin options. This method should be called
     * inside the implementation of the oppt::Plugin::load method. After
     * parsing the options file, the options object will be stored in
     * oppt::OpptPlugin::options_
     *
     * @param optionsFile A string containing the full path to the problem
     * configuration file
     */
    template<class OptionsType>
    void parseOptions_(const std::string optionsFile) {
        try {
            std::unique_ptr<OptionsType> options = std::make_unique<OptionsType>();
            PluginOptions* castedOptions = dynamic_cast<PluginOptions*>(options.get());
            if (!castedOptions)
                ERROR("Your plugin options type is not inherited from oppt::PluginOptions");
            options_ = std::move(options);
            std::unique_ptr <options::OptionParser> parser =
                OptionsType::makeParser();
            parser->setOptions(options_.get());
            parser->parseCfgFile(optionsFile);
            parser->finalize();
        } catch (const std::exception& e) {
            std::string msg = getFilename() + ": " + e.what();
            ERROR(msg);
        }
    }

private:
    typedef union {
        T* (*func)();
        void* ptr;
    } fptr_union_t;

    std::string handleName;

    std::string filename;

    void* dlHandle;

};

class RewardPlugin: public OpptPlugin<RewardPlugin>
{
public:
    friend class RobotEnvironment;    
    typedef std::unique_ptr<RewardPlugin> RewardPluginUniquePtr;

    _NO_COPY_OR_MOVE(RewardPlugin)

    RewardPlugin() {
        this->type_ = REWARD_PLUGIN;
    }

    virtual ~RewardPlugin() = default;

    /**
     * @brief Returns a reward given a oppt::PropagationResultSharedPtr
     */
    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const = 0;

    /**
     * @brief Get a pair of oppt::FloatType containing the minimum and maximum possible
     * immediate reward
     */
    virtual std::pair<FloatType, FloatType> getMinMaxReward() const = 0;

};

class HeuristicPlugin: public OpptPlugin<HeuristicPlugin>
{
public:
    friend class ProblemEnvironment;
    friend class solvers::Solver;    
    typedef std::unique_ptr<HeuristicPlugin> HeuristicPluginUniquePtr;
    typedef std::shared_ptr<HeuristicPlugin> HeuristicPluginSharedPtr;

    _NO_COPY_OR_MOVE(HeuristicPlugin)

    HeuristicPlugin() {
        this->type_ = HEURISTIC_PLUGIN;
    }

    virtual ~HeuristicPlugin() = default;

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const = 0;

};

class Robot;

class TransitionPlugin: public OpptPlugin<TransitionPlugin>
{
public:
    friend class Robot;
    typedef std::unique_ptr<TransitionPlugin> PropagatorPluginUniquePtr;   

    _NO_COPY_OR_MOVE(TransitionPlugin)

    TransitionPlugin() {
        this->type_ = TRANSITION_PLUGIN;
    }

    virtual ~TransitionPlugin() = default;


    /**
     * @brief Perfom forward propagation based on a PropagationRequest.
     * This method is the implicit definition of the POMDP transition function
     * P(s' | s, a).
     *
     * @param propagationRequest A pointer to the PropagationRequest
     *
     * @return A oppt::PropagationResultSharedPtr containing the propagated state
     */
    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const = 0;

    /**
     * @brief Returns a pointer to the underlying error distribution.
     * Implementing this method is optional, as per derfault, a nullptr is returned
     */    
    virtual Distribution<FloatType>* const getErrorDistribution() const {
        return nullptr;
    }
};

class ObservationPlugin: public OpptPlugin<ObservationPlugin>
{
public:
    friend class Robot;
    typedef std::unique_ptr<ObservationPlugin> ObservationPluginUniquePtr;

    _NO_COPY_OR_MOVE(ObservationPlugin)

    ObservationPlugin() {
        this->type_ = OBSERVATION_PLUGIN;
    }

    virtual ~ObservationPlugin() = default;

    /**
     * @brief Sample an observation based on the state contained in the ObservationRequest.
     * This method is the implicit definition of the POMDP observation function P(o | s, a)
     *
     * @param observationRequest A pointer to the ObservationRequest
     *
     * @return A oppt::ObservationResultSharedPtr to the ObservationResult
     */
    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const = 0;

    /**
     * @brief Calculate the likelihood of receiving the observation given the state
     */
    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const {
        return 1.0;
    }

    /**
     * @brief Returns a pointer to the underlying error distribution.
     * Implementing this method is optional, as per derfault, a nullptr is returned
     */
    virtual Distribution<FloatType>* const getErrorDistribution() const {
        return nullptr;
    }
};

class TerminalPlugin: public OpptPlugin<TerminalPlugin>
{
public:
    friend class RobotEnvironment;
    typedef std::unique_ptr<TerminalPlugin> TerminalPluginUniquePtr;

    _NO_COPY_OR_MOVE(TerminalPlugin)

    TerminalPlugin() {
        this->type_ = TERMINAL_PLUGIN;
    }

    virtual ~TerminalPlugin() = default;

    /**
     * @brief Checks wheter a PropagationResult is valid
     * @return A shared pointer to a ValidityReport object, containing information about the validity
     * of a PropagationResult
     */
    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) = 0;

    /**
     * @brief Checks wheter a PropagationResult leads to a terminal state
     * @return True if the resulting state inside the PropagationResult is terminal
     */
    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) = 0;
};

class InitialBeliefPlugin: public OpptPlugin<InitialBeliefPlugin>
{
public:
    friend class RobotEnvironment;
    typedef std::unique_ptr<InitialBeliefPlugin> InitialBeliefPluginUniquePtr;

    _NO_COPY_OR_MOVE(InitialBeliefPlugin)

    InitialBeliefPlugin() {
        this->type_ = INITIAL_BELIEF_PLUGIN;
    }

    virtual ~InitialBeliefPlugin() = default;

    /**
     * @brief Samples a state according to the initial belief
     */
    virtual RobotStateSharedPtr sampleAnInitState() = 0;
};

#define OPPT_REGISTER_REWARD_PLUGIN(classname) \
    extern "C" RewardPlugin *RegisterPlugin(); \
    RewardPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

#define OPPT_REGISTER_HEURISTIC_PLUGIN(classname) \
    extern "C" HeuristicPlugin *RegisterPlugin(); \
    HeuristicPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

#define OPPT_REGISTER_TRANSITION_PLUGIN(classname) \
    extern "C" TransitionPlugin *RegisterPlugin(); \
    TransitionPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

#define OPPT_REGISTER_OBSERVATION_PLUGIN(classname) \
    extern "C" ObservationPlugin *RegisterPlugin(); \
    ObservationPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

#define OPPT_REGISTER_TERMINAL_PLUGIN(classname) \
    extern "C" TerminalPlugin *RegisterPlugin(); \
    TerminalPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

#define OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(classname) \
    extern "C" InitialBeliefPlugin *RegisterPlugin(); \
    InitialBeliefPlugin *RegisterPlugin() \
    {\
        return new classname();\
    }

}

#endif

