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
#ifndef _PROBLEM_ENVIRONMENT_HPP_
#define _PROBLEM_ENVIRONMENT_HPP_
#include "oppt/global.hpp"
#include "ProblemEnvironmentOptions.hpp"
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/resources/resources.hpp"
#include "oppt/utils/include/filesystemUtils.hpp"
#include "oppt/plugin/Plugin.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImpl.hpp"

#include "oppt/solver/solver.hpp"
#include "ChangesParser.hpp"
#include "StepResult.hpp"
#include "SimulationResult.hpp"
#include <stdexcept>
#include <boost/timer.hpp>
#include "oppt/opptCore/Viewer.hpp"
#include <oppt/viewerPublisher/ViewerPublisher.hpp>

namespace oppt
{

/**
 * A parser that reads a configuration file, and constructs a ProblemEnvironmentOptions objects
 */
struct ProblemEnvironmentOptionsParser {
    template<typename OptionsType>
    ProblemEnvironmentOptionsPtr parseOptions(int argc, char const* argv[]) {
        std::unique_ptr<OptionsType> options_ = std::make_unique<OptionsType>();
        std::unique_ptr <options::OptionParser> parser =
            OptionsType::makeParser(true);

        std::string workingDir = oppt::get_current_directory();
        try {
            parser->setOptions(options_.get());
            parser->parseCmdLine(argc, argv);
            if (!options_->configPath.empty()) {
                parser->parseCfgFile(options_->configPath);
            }

            parser->finalize();
        } catch (options::OptionParsingException const& e) {
            std::cerr << e.what() << std::endl;
            return nullptr;
        }

        if (!resources::FileExists(options_->planningEnvironmentPath)) {
            ERROR("Environment file '" + options_->planningEnvironmentPath + "' doesn't exist");
        }

        if (!resources::FileExists(options_->executionEnvironmentPath)) {
            ERROR("Environment file '" + options_->executionEnvironmentPath + "' doesn't exist");
        }

        // Replace paths with their full path
        options_->planningEnvironmentPath = resources::FindFile(options_->planningEnvironmentPath);
        options_->executionEnvironmentPath = resources::FindFile(options_->executionEnvironmentPath);
        return std::move(options_);
    }
};


class ProblemEnvironment
{
public:
    ProblemEnvironment() {}

    virtual ~ProblemEnvironment() {}

    /**
     * The setup method that has to be called from the main method before running the environment
     */
    template<class SolverType, class OptionsType>
    int setup(int argc, char const* argv[]) {
        try {
            solver_ = std::make_unique<SolverType>();
            ProblemEnvironmentOptionsParser problemEnvironmentOptionsParser;
            problemEnvironmentOptions_ =
                problemEnvironmentOptionsParser.parseOptions<OptionsType>(argc, argv);
            if (!problemEnvironmentOptions_) {
                ERROR("Couldn't parse options");
            }

            setVerbose(problemEnvironmentOptions_->hasVerboseOutput);
            if (problemEnvironmentOptions_->seed == 0) {
                std::ifstream random("/dev/random", std::ios_base::in);
                int t;
                random.read(reinterpret_cast<char*>(&t), sizeof(t));
                problemEnvironmentOptions_->seed = t;
            }

            oppt::globalSeed = problemEnvironmentOptions_->seed;
            randGen_ = std::make_shared<RandomEngine>(problemEnvironmentOptions_->seed);
            randGen_->discard(100);
            if (problemEnvironmentOptions_->rngState > 0) {
                std::stringstream sstr;
                sstr << problemEnvironmentOptions_->rngState;
                sstr >> *(randGen_.get());
                cout << "Loaded PRNG state " << problemEnvironmentOptions_->rngState << endl;
            }

            // We need a custom callback to enable SDF to find our files
            std::function<std::string(const std::string&)> cb = [](const std::string & file) {
                VectorString elems;
                split(file, "model://", elems);
                if (elems.size() != 2)
                    ERROR("Could not resolve uri " + file);

                std::string modelName = elems[1];
                if (!resources::FolderExists(modelName))
                    ERROR("Model '" + modelName + "' doesn't exist");

                std::string modelFolder = resources::FindFolder(modelName);
                std::string modelConfigFileName = modelFolder + "/model.config";
                if (!resources::FileExists(modelConfigFileName))
                    ERROR("Model '" + modelName + "' doesn't have a valid model.config file");

                std::ifstream modelConfig(modelConfigFileName);
                std::string modelConfigString((std::istreambuf_iterator<char>(modelConfig)),
                                              std::istreambuf_iterator<char>());
                // Remove whitespace
                modelConfigString.erase(std::remove_if(modelConfigString.begin(), modelConfigString.end(), ::isspace), modelConfigString.end());
                std::string lookupString = modelName + "</name>";
                if (modelConfigString.find(lookupString) == std::string::npos)
                    ERROR("model.config for '" + modelName + "' doesn't reference '" + modelName + "'");

                unsigned first = modelConfigString.find("<sdf");
                unsigned last = modelConfigString.find("</sdf>");
                std::string modelFile = modelConfigString.substr(first, last - first);
                modelFile = modelFile.substr(modelFile.find(">") + 1);
                if (!resources::FileExists(modelFolder + "/" + modelFile))
                    ERROR("Robot model '" + modelFile + "' doesn't exist");
                std::string path = resources::FindFile(modelFile);
                VectorString elems2;
                split(path, modelFile, elems2);
                return elems2[0];
            };

            sdf::setFindCallback(cb);
            enableGazeboStateLogging(problemEnvironmentOptions_->enableGazeboStateLogging);

            //Now we're initializing the RobotEnvironments
            std::string planningPrefix = "planning";
            std::string executionPrefix = "exec";
            robotExecutionEnvironment_ =
                makeRobotEnvironment<RobotImpl, ProblemEnvironmentOptions>(problemEnvironmentOptions_.get(),
                        executionPrefix,
                        randGen_);
            robotExecutionEnvironment_->setExecutionEnvironment(true);
            robotPlanningEnvironment_ = makeRobotEnvironment<RobotImpl, ProblemEnvironmentOptions>(problemEnvironmentOptions_.get(),
                                        planningPrefix,
                                        randGen_);

            // Load the plugins
            //loadPlugins_();

            robotExecutionEnvironment_->getRobot()->initTransitionPlugin(problemEnvironmentOptions_->executionTransitionPlugin,
                    robotExecutionEnvironment_.get());
            robotExecutionEnvironment_->getRobot()->initObservationPlugin(problemEnvironmentOptions_->executionObservationPlugin,
                    robotExecutionEnvironment_.get());
            robotPlanningEnvironment_->getRobot()->initTransitionPlugin(problemEnvironmentOptions_->planningTransitionPlugin,
                    robotPlanningEnvironment_.get());
            robotPlanningEnvironment_->getRobot()->initObservationPlugin(problemEnvironmentOptions_->planningObservationPlugin,
                    robotPlanningEnvironment_.get());
            setupSpaces<ProblemEnvironmentOptions>(robotExecutionEnvironment_.get(), problemEnvironmentOptions_.get());
            setupSpaces<ProblemEnvironmentOptions>(robotPlanningEnvironment_.get(), problemEnvironmentOptions_.get());
            robotExecutionEnvironment_->getRobot()->loadTransitionPlugin(problemEnvironmentOptions_->configPath,
                    robotExecutionEnvironment_.get());
            robotExecutionEnvironment_->getRobot()->loadObservationPlugin(problemEnvironmentOptions_->configPath,
                    robotExecutionEnvironment_.get());
            robotPlanningEnvironment_->getRobot()->loadTransitionPlugin(problemEnvironmentOptions_->configPath,
                    robotPlanningEnvironment_.get());
            robotPlanningEnvironment_->getRobot()->loadObservationPlugin(problemEnvironmentOptions_->configPath,
                    robotPlanningEnvironment_.get());
            loadTerminalPlugin_("planning");
            loadTerminalPlugin_("exec");
            loadInitialBeliefPlugin_("planning");
            loadInitialBeliefPlugin_("exec");
            loadRewardPlugin_("planning");
            loadRewardPlugin_("execution");

            // Parse the changes
            parseChanges();

            // Setup the solver here
            setupSolver_();
            auto changeHandlerFunction = ProblemEnvironmentChangeHandlerFn(std::bind(&ProblemEnvironment::handleEnvironmentChanges,
                                         this,
                                         std::placeholders::_1));
            robotExecutionEnvironment_->setProblemEnvironmentChangeHandlerFn(changeHandlerFunction);

            std::string resultsDir = problemEnvironmentOptions_->logPath;

            onStepFinishedFn_ = [this, resultsDir](const unsigned int & run, const unsigned int & step) {
                // Add planned changes here for this step
                // robotExecutionEnvironment_->addChanges();
                if (changesMap_.find(step) == changesMap_.end())
                    changesMap_[step] = environmentUserChanges_;
                else
                    changesMap_.at(step).insert(changesMap_.at(step).end(),
                                                environmentUserChanges_.begin(),
                                                environmentUserChanges_.end());

                // Add changes to robot environment
                // robotExecutionEnvironment_->addChanges();
                for (auto & change : changesMap_.at(step)) {
                    WARNING("Add environment change");
                    robotExecutionEnvironment_->addEnvironmentChange(change);
                }


                // Apply all changes
                robotExecutionEnvironment_->applyChanges();

                // Inform the solver
                if (changesMap_.at(step).size() > 0)
                    solver_->handleEnvironmentChanges(changesMap_.at(step));

                // Serialize the changes made by the user
                if (environmentUserChanges_.size() > 0) {
                    std::string changesFile = resultsDir + "/changes_" + std::to_string(run) + ".txt";
                    std::ofstream os(changesFile,
                                     std::ios_base::app | std::ios_base::out);
                    for (auto & environmentChange : environmentUserChanges_) {
                        os << "t=" << step << " ";
                        environmentChange->serialize(os);
                    }

                    os.close();
                    environmentUserChanges_.clear();
                }
            };

            onRunFinishedFn_ = [this](const unsigned int & run) {
                // Undo the previous changes and reload the planned changes
                changesMap_.clear();
                parseChanges();

                resetRobotEnvironment<RobotImpl, ProblemEnvironmentOptions>(robotExecutionEnvironment_.get(),
                        problemEnvironmentOptions_.get());
                resetRobotEnvironment<RobotImpl, ProblemEnvironmentOptions>(robotPlanningEnvironment_.get(),
                        problemEnvironmentOptions_.get());
            };

            return 0;
        } catch (const std::runtime_error& err) {
            std::cerr << err.what() << std::endl;
            return 2;
        } catch (const options::OptionParsingException& err) {
            std::cerr << err.what() << std::endl;
            return 2;
        }
    }

    virtual void handleEnvironmentChanges(const EnvironmentChangeSharedPtr& environmentChange) {
        environmentUserChanges_.push_back(environmentChange);
    }

    /**
     * @brief Run the environment. This method should be called from the main method after setup has been called.
     */
    virtual int runEnvironment(int argc, char const* argv[]) {
        try {
            if (!problemEnvironmentOptions_) {
                cout << "Error: You can't run an environment before calling the setup() method" << endl;
                return 2;
            }

            std::string resultsDir = problemEnvironmentOptions_->logPath;
            if (!oppt::createDir(resultsDir)) {
                cout << "Error: results directory couldn't be created: " << resultsDir << endl;
                return 2;
            }

            std::string finalLogFilePath = getLogFilePath(resultsDir);
            if (!oppt::fileExists(finalLogFilePath)) {
                std::ofstream os(finalLogFilePath,
                                 std::ios_base::app | std::ios_base::out);

                // Check the validity of the goal states
                //goalStatesValid();

                // Put some initial information into the logfile
                os << "seed: " << problemEnvironmentOptions_->seed << endl;
                //os << "Process error: " << problemEnvironmentOptions_->processError << endl;
                //os << "Observation error: " << problemEnvironmentOptions_->observationError << endl;
                os << "Robot: " << robotPlanningEnvironment_->getRobot()->getName() << endl;
                os << "Planning environment: " << problemEnvironmentOptions_->planningEnvironmentPath << endl;
                os << "Execution environment: " << problemEnvironmentOptions_->executionEnvironmentPath << endl;
                os << "solver: " << solver_->getName() << endl;

                FloatType totalDiscountedReward = 0;
                unsigned int totalNumSteps = 0;
                unsigned int meanNumSteps = 0;
                FloatType meanPlanningTimePerStep = 0;
                unsigned int numSuccessfulRuns = 0;
                for (size_t i = 0; i != problemEnvironmentOptions_->nRuns; ++i) {
                    os << "Run #" << i + 1 << endl;
                    cout << "Run # " << i + 1 << endl;
                    SimulationResult simulationResult = run(i + 1, os, argc, argv);
                    totalDiscountedReward += simulationResult.discountedReward;
                    totalNumSteps += simulationResult.stepsTaken;
                    meanNumSteps += simulationResult.stepsTaken;
                    meanPlanningTimePerStep += simulationResult.totalPlanningTime;
                    if (simulationResult.successfulRun)
                        numSuccessfulRuns++;
                    onRunFinishedFn_(i + 1);

                    os << "RUN_FINISHED_USER_DATA_BEGIN" << endl;
                    // We're done with this run. Let the solver know about it
                    solver_->runFinished(os, i + 1);
                    os << "RUN_FINISHED_USER_DATA_END" << endl;
                    cout << "Run finished \n";
                    cout << "Discounted reward: " << simulationResult.discountedReward << endl;
                }

                meanNumSteps /= (FloatType)problemEnvironmentOptions_->nRuns;
                meanPlanningTimePerStep /= (FloatType)totalNumSteps;

                FloatType percentageSuccRuns =
                    (100.0 / (FloatType)problemEnvironmentOptions_->nRuns) * (FloatType)numSuccessfulRuns;

                os << "##################################\n";
                os << "Mean number of steps: " << meanNumSteps << endl;
                os << "Mean planning time per step: " << meanPlanningTimePerStep * 1000 << "ms\n";
                os << "Num successful runs: " << numSuccessfulRuns << endl;
                os << "Percentage of successful runs: " << percentageSuccRuns << endl;
                os << "Mean rewards: " << totalDiscountedReward / (FloatType)problemEnvironmentOptions_->nRuns << endl;

                os.close();
            } else {
                oppt::LOGGING("A logfile with the same covariance parameters exists. Skipping");
            }


            oppt::LOGGING("Done.");
            return 0;
        } catch (const std::runtime_error& re) {
            return 2;
        }
    }

    /**
     * @brief Returns a pointer to the ProblemEnvironmentOptions object
     */
    const ProblemEnvironmentOptions* getOptions() const {
        return problemEnvironmentOptions_.get();
    }

    /**
     * @brief Returns a pointer to the robot execution environment
     */
    RobotEnvironment *getRobotExecutionEnvironment() const {
        if (!robotExecutionEnvironment_)
            ERROR("IS NULL!!!!");
        return robotExecutionEnvironment_.get();
    }

    RobotEnvironment *getRobotPlanningEnvironment() const {
        return robotPlanningEnvironment_.get();
    }

    solvers::Solver* getSolver() const {
        return solver_.get();
    }

    std::string getLogFilePath(const std::string& resultsDir) {
        unsigned int i = 0;
        std::string envName = "";
        std::string filename =
            "log_" + solver_->getName() +
            envName +
            "_" + robotExecutionEnvironment_->getRobot()->getName() +
            "_" + problemEnvironmentOptions_->logFilePostfix +
            ".log";
        std::string path = resultsDir + "/" + filename;
        if (oppt::fileExists(path) && problemEnvironmentOptions_->overwriteExistingLogFiles) {
            oppt::removeFile(path);
        }

        return path;
    }

    void updateViewer(const RobotStateSharedPtr& currentState,
                      const VectorRobotStatePtr& particles = VectorRobotStatePtr()) {
        if (!currentState) {
            LOGGING("currentState is null. Cannot visualize current simulation state");
            return;
        }
        VectorRobotStatePtr subStates = currentState->getSubStates();
        unsigned int numDrawnParticles = problemEnvironmentOptions_->particlePlotLimit;
        if (particles.size() < numDrawnParticles)
            numDrawnParticles = particles.size();
        for (size_t i = 0; i != subStates.size(); ++i) {
            FloatType d = ((1.0 / (FloatType)(subStates.size())) * i);
            VectorRobotStatePtr particlesVec;
            for (size_t j = 0; j != numDrawnParticles; ++j) {
                auto opptBeliefState = particles[j];
                if (opptBeliefState) {
                    auto userData = opptBeliefState->getUserData();
                    if (userData) {
                        auto previousParticleState = userData->as<RobotStateUserData>()->previousState;
                        auto v = static_cast<const VectorState*>(previousParticleState.get())->asVector();
                        oppt::RobotStateSharedPtr prevS(new oppt::VectorState(v));
                        auto interpolatedState = robotPlanningEnvironment_->getRobot()->getStateSpace()->interpolate(prevS,
                                                 opptBeliefState,
                                                 d);
                        particlesVec.push_back(interpolatedState);
                    }
                }
            }

            auto normalizedSubState =
                robotPlanningEnvironment_->getRobot()->getStateSpace()->normalizeState(subStates[i]);

            robotExecutionEnvironment_->getRobot()->updateViewer(normalizedSubState,
                    particlesVec,
                    problemEnvironmentOptions_->particleOpacity,
                    false,
                    false);
        }

        VectorRobotStatePtr particlesVec(numDrawnParticles);
        for (size_t i = 0; i != numDrawnParticles; ++i) {
            if (i < particles.size())
                particlesVec[i] = particles[i];
        }

        auto environmentInfo = robotExecutionEnvironment_->getEnvironmentInfo();
        static_cast<ViewerPublisher *>(robotExecutionEnvironment_->getRobot()->getViewer())->updateFromEnvironmentInfo(environmentInfo, true);

        robotExecutionEnvironment_->getRobot()->updateViewer(currentState,
                particlesVec,
                problemEnvironmentOptions_->particleOpacity,
                false,
                true);
    }

protected:
    RandomEnginePtr randGen_;

    ProblemEnvironmentOptionsPtr problemEnvironmentOptions_;

    solvers::SolverPtr solver_;

    std::unique_ptr<RobotEnvironment> robotPlanningEnvironment_;

    std::unique_ptr<RobotEnvironment> robotExecutionEnvironment_;

    VectorEnvironmentChanges environmentUserChanges_;

    std::unordered_map<unsigned int, VectorEnvironmentChanges> changesMap_;

private:
    std::function<void(const unsigned int&, const unsigned int&)> onStepFinishedFn_ = nullptr;

    std::function<void(const unsigned int&)> onRunFinishedFn_ = nullptr;

private:
    void registerForEnvironmentChanges(RobotEnvironment* const registeredEnvironment,
                                       RobotEnvironment* const environment) {
        if (!environment)
            robotExecutionEnvironment_->registerForEnvironmentChanges(registeredEnvironment);
        else
            environment->registerForEnvironmentChanges(registeredEnvironment);
    }

    bool unregisterForEnvironmentChanges(RobotEnvironment* const registeredEnvironment,
                                         RobotEnvironment* const environment) {
        if (!environment)
            return robotExecutionEnvironment_->unregisterForEnvironmentChanges(registeredEnvironment);
        else
            return environment->unregisterForEnvironmentChanges(registeredEnvironment);
    }

    void setStateNormalizer(std::unique_ptr<StateNormalizer> stateNormalizer, const bool &executionEnvironment) {
        if (executionEnvironment)
            robotExecutionEnvironment_->getRobot()->getStateSpace()->setStateNormalizer(std::move(stateNormalizer));
        else
            robotPlanningEnvironment_->getRobot()->getStateSpace()->setStateNormalizer(std::move(stateNormalizer));
    }

    void setActionNormalizer(std::unique_ptr<ActionNormalizer> actionNormalizer, const bool &executionEnvironment) {
        if (executionEnvironment)
            robotExecutionEnvironment_->getRobot()->getActionSpace()->setActionNormalizer(std::move(actionNormalizer));
        else
            robotPlanningEnvironment_->getRobot()->getActionSpace()->setActionNormalizer(std::move(actionNormalizer));

    }

    void setObservationNormalizer(std::unique_ptr<ObservationNormalizer> observationNormalizer, const bool &executionEnvironment) {
        if (executionEnvironment)
            robotExecutionEnvironment_->getRobot()->getObservationSpace()->setObservationNormalizer(std::move(observationNormalizer));
        else
            robotPlanningEnvironment_->getRobot()->getObservationSpace()->setObservationNormalizer(std::move(observationNormalizer));
    }

    void parseChanges() {
        if (problemEnvironmentOptions_->hasChanges) {
            if (!oppt::resources::FileExists(problemEnvironmentOptions_->changesPath)) {
                WARNING("Path for changes '" + problemEnvironmentOptions_->changesPath + "' doesn't exist. Ignoring");
            } else {
                ChangesParser changesParser;
                std::string changesPath = resources::FindFile(problemEnvironmentOptions_->changesPath);
                changesParser.parseChanges(changesPath, changesMap_);
            }
        }
    }

    void setupSolver_() {
        solver_->setRobotPlanningEnvironment(robotPlanningEnvironment_.get());
        solver_->setProblemEnvironmentOptions(problemEnvironmentOptions_.get());
        if (problemEnvironmentOptions_->heuristicPlugin == "")
            ERROR("'heuristicPlugin' option is empty");
        solver_->loadHeuristicPlugin(problemEnvironmentOptions_->heuristicPlugin);
        solver_->setRandomEngine(randGen_);

        auto registerEnvironmentFn =
            solvers::RegisterEnvironmentFn(std::bind(&ProblemEnvironment::registerForEnvironmentChanges,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
        auto unregisterEnvironmentFn =
            solvers::UnregisterEnvironmentFn(std::bind(&ProblemEnvironment::unregisterForEnvironmentChanges,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
        solver_->setRegisterEnvironmentFn(registerEnvironmentFn);
        solver_->setUnregisterEnvironmentFn(unregisterEnvironmentFn);

        auto setStateNormalizerFn = solvers::SetStateNormalizerFn(std::bind(&ProblemEnvironment::setStateNormalizer,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
        auto setActionNormalizerFn = solvers::SetActionNormalizerFn(std::bind(&ProblemEnvironment::setActionNormalizer,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2));
        auto setObservationNormalizerFn = solvers::SetObservationNormalizerFn(std::bind(&ProblemEnvironment::setObservationNormalizer,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
        solver_->setSetStateNormalizerFn(setStateNormalizerFn);
        solver_->setSetActionNormalizerFn(setActionNormalizerFn);
        solver_->setSetObservationNormalizerFn(setObservationNormalizerFn);
        solver_->setup();
    }

    void loadTerminalPlugin_(const std::string prefix) {
        if (prefix == "planning") {
            if (!resources::FileExists(problemEnvironmentOptions_->planningTerminalPlugin)) {
                ERROR("Terminal plugin '" + problemEnvironmentOptions_->planningTerminalPlugin +
                      "' doesn't exist");
            }

            robotPlanningEnvironment_->loadTerminalPlugin(problemEnvironmentOptions_->planningTerminalPlugin,
                    problemEnvironmentOptions_->configPath);
        } else {
            if (!resources::FileExists(problemEnvironmentOptions_->executionTerminalPlugin)) {
                ERROR("Terminal plugin '" + problemEnvironmentOptions_->executionTerminalPlugin +
                      "' doesn't exist");
            }

            robotExecutionEnvironment_->loadTerminalPlugin(problemEnvironmentOptions_->executionTerminalPlugin,
                    problemEnvironmentOptions_->configPath);
        }
    }

    void loadRewardPlugin_(const std::string prefix) {
        if (prefix == "planning") {
            if (!resources::FileExists(problemEnvironmentOptions_->planningRewardPlugin)) {
                ERROR("Reward plugin '" + problemEnvironmentOptions_->planningRewardPlugin + "' doesn't exist");
            }

            robotPlanningEnvironment_->loadRewardPlugin(problemEnvironmentOptions_->planningRewardPlugin,
                    problemEnvironmentOptions_->configPath);
        } else {
            if (!resources::FileExists(problemEnvironmentOptions_->executionRewardPlugin)) {
                ERROR("Reward plugin '" + problemEnvironmentOptions_->executionRewardPlugin + "' doesn't exist");
            }

            robotExecutionEnvironment_->loadRewardPlugin(problemEnvironmentOptions_->executionRewardPlugin,
                    problemEnvironmentOptions_->configPath);
        }
    }

    void loadInitialBeliefPlugin_(const std::string& prefix) {
        if (prefix == "planning") {
            if (!resources::FileExists(problemEnvironmentOptions_->initialBeliefPluginPlanning)) {
                ERROR("Initial state sampler plugin '" + problemEnvironmentOptions_->initialBeliefPluginPlanning +
                      "' doesn't exist");
            }

            robotPlanningEnvironment_->loadInitialBeliefPlugin(problemEnvironmentOptions_->initialBeliefPluginPlanning,
                    problemEnvironmentOptions_->configPath);
        } else {
            if (!resources::FileExists(problemEnvironmentOptions_->initialBeliefPluginExecution)) {
                ERROR("Initial state sampler plugin '" + problemEnvironmentOptions_->initialBeliefPluginExecution +
                      "' doesn't exist");
            }

            robotExecutionEnvironment_->loadInitialBeliefPlugin(problemEnvironmentOptions_->initialBeliefPluginExecution,
                    problemEnvironmentOptions_->configPath);
        }
    }

    template<class RobotType, class OptionsType>
    std::unique_ptr<oppt::RobotEnvironment> makeRobotEnvironment(OptionsType* options,
            std::string& prefix,
            std::shared_ptr<std::default_random_engine> randomEngine,
            const unsigned int& threadId = 0) {
        std::unique_ptr<oppt::RobotEnvironment> robot_environment = std::make_unique<oppt::RobotEnvironment>();
        robot_environment->setRandomEngine(randomEngine);
        bool created = false;
        bool loaded_environment = false;
        std::string robotName;
        if (prefix == "planning") {
            created = robot_environment->createRobot<RobotType>(options->planningEnvironmentPath,
                      options->robotName,
                      options->configPath,
                      prefix,
                      threadId);
            robotName = robot_environment->getRobot()->getName();
            loaded_environment = robot_environment->loadEnvironment(options->planningEnvironmentPath, robotName);
            robot_environment->getRobot()->setCollisionsAllowed(options->allowCollisions);
        } else if (prefix == "exec") {
            created = robot_environment->createRobot<RobotType>(options->executionEnvironmentPath,
                      options->robotName,
                      options->configPath,
                      prefix);
            robotName = robot_environment->getRobot()->getName();
            loaded_environment = robot_environment->loadEnvironment(options->executionEnvironmentPath, robotName);
            robot_environment->getRobot()->setCollisionsAllowed(options->allowCollisions);
            if (!(options->deactivateVisualization)) {
                robot_environment->getRobot()->initializeViewer_(options->robotName,
                        options->executionEnvironmentPath);
            }
        }

        if (!loaded_environment)
            ERROR("Environment couldn't be loaded");
        return std::move(robot_environment);
    }

    template<class RobotType, class OptionsType>
    void resetRobotEnvironment(oppt::RobotEnvironment* robotEnvironment,
                               const OptionsType* options) {
        std::string robotName = robotEnvironment->getRobot()->getName();
        bool loadedEnvironment = robotEnvironment->loadEnvironment(options->executionEnvironmentPath,
                                 robotName,
                                 true);
        if (!loadedEnvironment)
            ERROR("Couldn't load environment");
        robotEnvironment->makeEnvironmentInfo(options->interactive);
        for (auto & env : robotEnvironment->getRegisteredRobotEnvironments()) {
            resetRobotEnvironment<RobotType, OptionsType>(env, options);
        }
    }

    template<class OptionsType>
    void setupSpaces(RobotEnvironment* robotEnvironment, OptionsType* options) {
        robotEnvironment->getRobot()->init();
        StateSpaceInfo stateSpaceInfo;
        ObservationSpaceInfo observationSpaceInfo;
        ActionSpaceInfo actionSpaceInfo;
        std::vector<oppt::RobotStateSharedPtr> goalStates;
        oppt::Robot* robot = robotEnvironment->getRobot();

        robot->normalizedSpaces_ = options->normalizedSpaces;

        // Make state space
        stateSpaceInfo.normalized = options->normalizedSpaces;
        robot->makeStateSpace(stateSpaceInfo);

        // Make observation space
        observationSpaceInfo.normalized = options->normalizedSpaces;
        robot->makeObservationSpace(observationSpaceInfo);

        // Make action space
        actionSpaceInfo.normalized = options->normalizedSpaces;

        robot->makeActionSpace(actionSpaceInfo);
        robotEnvironment->makeEnvironmentInfo(options->interactive);
        robotEnvironment->initializeEnvironmentCallbacks();
    }

    void serializeBeliefParticles(std::ofstream& os, const VectorRobotStatePtr& beliefParticles) {
        bool logTmp = logGazeboState;
        bool gazeboStateLogging = false;
        enableGazeboStateLogging(gazeboStateLogging);
        os << "PARTICLES BEGIN" << endl;
        for (size_t i = 0; i != beliefParticles.size(); ++i) {
            beliefParticles[i]->serialize(os, "p");
            os << " \n";
        }
        os << "PARTICLES END" << endl;
        enableGazeboStateLogging(logTmp);
    }

    SimulationResult run(const unsigned int& run, std::ofstream& os, int argc, char const* argv[]) {
        SimulationResult simulationResult;
        if (!solver_->reset())
            return simulationResult;

        // Apply possible changes made to the execution environment
        // during the setup process
        robotExecutionEnvironment_->applyChanges();

        LOGGING("Wait for input after reset");
        //boost::this_thread::sleep_for(boost::chrono::milliseconds((unsigned int)7000));


        unsigned int currentStep = 0;
        FloatType totalDiscountedReward = 0;
        FloatType currentDiscount = 1.0;
        FloatType totalPlanningTime = 0;
        bool success = false;

        RobotStateSharedPtr currentState =
            robotExecutionEnvironment_->sampleInitialState();
        if (problemEnvironmentOptions_->hasVerboseOutput)
            cout << "Initial state: " <<
                 *(robotExecutionEnvironment_->getRobot()->getStateSpace()->denormalizeState(currentState).get()) << endl;
        RobotStateSharedPtr finalState;
        StepResult stepResult(0, robotExecutionEnvironment_.get());
        VectorRobotStatePtr beliefParticles;
        auto viewer = robotExecutionEnvironment_->getRobot()->getViewer();
        if (viewer && viewer->viewerRunning())
            updateViewer(currentState);
        auto robot = robotExecutionEnvironment_->getRobot();
        bool serializeLastBelief = true;

        boost::timer t0;
        while (true) {
            cout << "\nt = " << currentStep << endl;
            stepResult = StepResult(currentStep, robotExecutionEnvironment_.get());
            stepResult.currentState = currentState;
            stepResult.discountFactor = currentDiscount;
            if (currentStep >= problemEnvironmentOptions_->nSimulationSteps) {
                // We're out of steps
                PRINT("Out of steps. Ending simulation run");
                finalState = currentState;
                break;
            }

            if (problemEnvironmentOptions_->hasVerboseOutput)
                cout << "Improving policy...\n";

            // Improve the current policy
            boost::timer planningTimer;
            if (!solver_->improvePolicy(problemEnvironmentOptions_->stepTimeout)) {
                finalState = currentState;
                totalPlanningTime += planningTimer.elapsed();
                LOGGING("Couldn't improve policy. Ending simulation run");
                break;
            }

            totalPlanningTime += planningTimer.elapsed();

            // Get nextAction
            ActionSharedPtr action = solver_->getNextAction();
            if (!action) {
                // No action to execute
                finalState = currentState;
                LOGGING("No action for the next step. Ending simulation run");
                break;
            }

            cout << "action: " <<
                 *(robotPlanningEnvironment_->getRobot()->getActionSpace()->denormalizeAction(action))
                 << endl;
            stepResult.action = action;

            // Execute action
            PropagationRequestSharedPtr propagationRequest(new PropagationRequest());
            propagationRequest->currentState = currentState;
            propagationRequest->action = action.get();
            propagationRequest->allowCollisions = problemEnvironmentOptions_->allowCollisions;

            oppt::PropagationResultSharedPtr propagationResult =
                robotExecutionEnvironment_->getRobot()->propagateState(propagationRequest);
            if (propagationResult->nextState) {
                cout << "Next state: " <<
                     *(robotPlanningEnvironment_->getRobot()->getStateSpace()->denormalizeState(propagationResult->nextState).get())
                     << endl;
            } else {
                cout << "Next state: NULL" << endl;
            }

            // Get an observation
            ObservationRequestSharedPtr observationRequest(new ObservationRequest());
            observationRequest->currentState = propagationResult->nextState;
            observationRequest->action = action.get();
            ObservationResultSharedPtr observationResult =
                robotExecutionEnvironment_->getRobot()->makeObservationReport(observationRequest);

            stepResult.observation = observationResult->observation;
            auto observationSpace = robotPlanningEnvironment_->getRobot()->getObservationSpace();
            cout << "Observation: " <<
                 *(observationSpace->denormalizeObservation(stepResult.observation).get()) <<
                 endl;

            // Get a reward
            FloatType reward = robotExecutionEnvironment_->getReward(propagationResult);

            totalDiscountedReward += currentDiscount * reward;
            stepResult.reward = reward;
            cout << "Immediate reward: " << reward << endl;

            // Update the true state
            currentState = propagationResult->nextState;
            currentDiscount *= problemEnvironmentOptions_->discountFactor;

            // Check if we're terminal
            bool terminal = robotExecutionEnvironment_->isTerminal(propagationResult);

            if (terminal) {
                PRINT("Terminal state reached!");
                finalState = currentState;
                if (reward == robotExecutionEnvironment_->getRewardPlugin()->getMinMaxReward().second)
                    success = true;
            }

            // Update the belief
            if (!solver_->updateBelief(action, observationResult->observation, terminal)) {
                // Couldn't update the belief
                finalState = currentState;
                serializeLastBelief = false;
                LOGGING("Couldn't update belief. Ending simulation run");
                break;
            }

            if (problemEnvironmentOptions_->hasVerboseOutput)
                PRINT("Updated belief");

            beliefParticles = solver_->getBeliefParticles();

            // Update the viewer
            if (viewer && viewer->viewerRunning())
                updateViewer(currentState, beliefParticles);


            // Serialization
            stepResult.serialize(os);
            if (problemEnvironmentOptions_->saveParticles)
                serializeBeliefParticles(os, beliefParticles);
            os << "SOLVER_DATA_BEGIN\n";
            solver_->serializeStep(os);
            os << "SOLVER_DATA_END\n";
            solver_->stepFinished(currentStep);

            onStepFinishedFn_(run, currentStep);

            if (terminal)
                break;

            // Update step
            currentStep++;
        }

        FloatType totalTimeTaken = t0.elapsed();

        stepResult.serialize(os);

        //finalState = stepResult.currentState;
        os << "FINAL_STATE_BEGIN\n";
        if (finalState)
            finalState->serialize(os, "S");
        os << "\n";
        os << "FINAL_STATE_END\n";
        if (serializeLastBelief && problemEnvironmentOptions_->saveParticles) {
            beliefParticles = solver_->getBeliefParticles();
            serializeBeliefParticles(os, beliefParticles);
        }
        os << "Total discounted reward: " << totalDiscountedReward << endl;
        if (success)
            os << "Run successful: True\n";
        else
            os << "Run successful: False\n";
        os << "Num steps: " << currentStep << endl;
        os << "Total time taken: " << totalTimeTaken * 1000.0 << "ms" << endl;

        simulationResult.discountedReward = totalDiscountedReward;
        simulationResult.stepsTaken = currentStep;
        simulationResult.totalPlanningTime = totalPlanningTime;
        simulationResult.successfulRun = success;

        //solver_->run(os, argc, argv);
        return simulationResult;
    }

};
}

#endif
