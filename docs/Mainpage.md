OPPT
==========================================================================

[TOC]

# Introduction {#intro}
On-line POMDP Planning Toolkit (OPPT) is a C++ software-toolkit for approximating POMDP solutions on-line. To ease implementation of POMDP models, OPPT allows users to specify the state, action, and observation spaces via a configuration file, and uses a plug-in architecture to implement the core components of a POMDP, that is, the transition, observation and reward functions. 
For standard motion planning under partial observability problems - that is, moving from one configuration to another with errors in the effect of actions and sensing -, OPPT 
provides a default POMDP model, such that users only need to specify 3D models of the robot and environment, and a configuration file that specifies parameters for the probability density functions that represent the uncertainty in the effect of actions, observations, and starting state, and the reward function.

OPPT also allows a user to separate the POMDP model (including the robot’s environment) for planning and for simulated execution. It is known that developing a faithful
POMDP model is often the main bottleneck in applications of POMDP based planning, today. However, it is also known that strategies computed with imperfect POMDP models can
still generate relatively good robot behaviours. The ability to separate planning and execution environments will better facilitate sensitivity analysis studies of on-line POMDP solvers
and allow users to better predict the performance of on-line POMDP solvers in the physical world. For testing new solvers, OPPT provides an abstract and
general POMDP solver class that is not restricted to specific data structures. The user has access to a rich framework that provides functionalities common for many motion planning
problems, such as kinematic computations, physical simulation of the robot and the environment it operates in and collision detection. This enables the user to focus on implementing
new POMDP solvers

# System Requirements {#requirements}

- OS: Ubuntu 15.10 or higher
- [GNU C++ compiler](https://gcc.gnu.org) (>= 4.9.4) or equivalent

# Dependencies {#dependencies}

- Gazebo v7.1.0 or higher and its dependencies for physics simulation and kinematic computations
- Eigen v3.2.0 or higher for linear algebra computations
- FCL v0.4.0 or higher for collision queries
- Assimp v3.3.1 or higher for importing meshes
- Boost v1.55.0 or higher (required packages: system, thread, timer, filesystem, serialization, program_options, signals, unit_test_framework)
- TinyXML v2.6.2 or higher
- spatialindex v1.8.5 or higher
- (optional) ROS Kinetic or higher for visualization

## Installing the dependencies

### One-line installation

Before building OPPT, you have to make sure that all the dependencies are installed. On a clean Ubuntu installation, use the following command to install the dependencies:

    wget https://bitbucket.org/hoergems/oppt_scripts/downloads/install_dependencies.sh && chmod +x install_dependencies.sh && ./install_dependencies.sh
    
### Step-by-step installation

1. Install the core dependencies

        sudo apt-get install build-essential git cmake mercurial pkg-config libboost-all-dev libtinyxml-dev libeigen3-dev libassimp-dev libfcl-dev
        
2. Install Gazebo and it's dependencies by following the instructions on the [Gazebo website](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
        
3. Download and install libspatialindex-1.8.5

        wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz
        tar zxfv spatialindex-src-1.8.5.tar.gz
        cd spatialindex-src-1.8.5
        mkdir build && cd build
        cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
        make -j2 && sudo make install
        
4. (Optional) If you want to use the GUI provided in OPPT or the Inverse Kinematics functionality, you need to have a working installation of ROS (at least the ROS-base stack along with the rviz, kdl-parser and trac-ik stack). Furthermore your ROS environment has to be set up (please refer to http://wiki.ros.org/<ROS DISTRIBUTION>/Installation/Ubuntu section 1.6). If you do not have ROS installation, you can install it using

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-get update
        sudo apt-get install ros-<ROS DISTRIBUTION>-ros-base ros-<ROS DISTRIBUTION>-rviz ros-<ROS DISTRIBUTION>-kdl-parser ros-<ROS DISTRIBUTION>-trac-ik ros-<ROS DISTRIBUTION>-trac-ik-lib

where *<ROS_DISTRIBUTION>* is the ROS distribution corresponding to your Ubuntu version.
        
<a id="building"></a>
# Building OPPT {#building}
        
6. Building and installing OPPT

        cd <oppt_root_folder>/src/
        mkdir build && cd build
        cmake -DCMAKE_BUILD_TYPE=<BUILD_TYPE> -DCMAKE_INSTALL_PREFIX=<oppt_install_location> ..
        make && make install
        
OPPT supports three build types: Debug, RelWithDebInfo and Release. The default build type is RelWithDebInfo

# Configuring the OPPT runtime environment {#configRuntime}

In order to obtain resources such as SDF models, plugins, etc., OPPT uses a filesystem API

    oppt::FileExists("filename")
    oppt::FindFile("filename")
    
which locates resources inside folders specified in the 

    OPPT_RESOURCE_PATH
    
environment variable. To configure your current environment, either call

    source <oppt_install_location>/share/oppt/setup.sh
    
or add this line to your bash environment file
    
This will add the 

    <oppt_install_location>/share/oppt/models
    <oppt_install_location>/share/oppt/plugins

folders to the resource environment variable. If you want to add additional folders, use

    export OPPT_RESOURCE_PATH=$OPPT_RESOURCE_PATH:<additional_folder>
    
If you have a working ROS installation, make sure that the ROS environment is set-up properly.
For a clean ROS installation, this is done via

    source /opt/ros/<ROS DISTRIBUTION>/setup.sh
    
If you fail to do this, you will get an 'ROS_MASTER_URI is not defined' error

# Quick start {#quickStart}

After OPPT was compiled successfully, navigate to the

    <oppt_root_folder>/bin
    
directory. Here you should see the 'abt' executable. If you have installed the optional ROS dependency, you should see two additional executables, 'viewer' and 'playVideo'.
The problem configuration files for the problems that ship with OPPT are inside the

    <oppt_root_folder>/cfg
    
directory. To run the ABT solver for any of the problems, run

    ./abt --cfg <oppt_root_folder>/cfg/<ProblemConfigurationFile>.cfg
    
Note that the path to the problem configuration file must be an absolute path.

If you want a visualization of the simulation runs, open a separate terminal and run

    opptViewer
    
BEFORE running the ABT solver.

# Defining a motion planning problem {#motionPlanning}
Before showing how to implement a specific POMDP model using the plugin architecture, we show how to define and set-up motion planning problems under uncertainty for which the standard plugin implementations can be used.
In OPPT motion planning problems under uncertainty are a class of problems in which a robot starts from an initial belief and has to reach a goal area within the environment while avoiding collisions with static obstacles that are present within the environment. It is assumed that the robot consists of a set of links connected by revolute torque-controlled joints. Additionally the input torques of the joints are disturbed by additive multivariate Gaussian noise. Similarly the observations the robot receives are affected by additive multivariate Gaussian noise as well. For this class of problems, terminal states are states in which the robot either collides with an obstacle or reaches the goal area. For collisions with the obstacles the robot receives a user-defined penalty, for reaching the goal area it receives a user-defined reward. Additionally the robot receives a small penalty for every step it takes.
If a planning problem under uncertainty falls within this class of problems, the standard plugins can be used. In this example we will show how to define a POMDP problem in which a 2DOF torque controlled manipulator that operates inside a 3D environment has to reach a goal area without colliding with an obstacle.

### Environment and robot model

OPPT follows the Gazebo convention in defining environment and robot models, in particular using the SDF format to define models. A typical folder structure for environment and robot models looks as follows:

- *ModelFolder* (this folder has to be added to the OPPT_RESOURCE_PATH environment variable)
    - *MyModelFolder*
        - *MyModel.sdf*
        - *model.config*
        
The *MyModel.sdf* is the SDF file that describes your environment or robot model, whereas the *model.config* contains meta-information about your model and should have the following structure:

    <?xml version="1.0"?>
    <model>
      <name>MyModel</name>
      <version>1.0</version>
      <sdf version='1.6'>MyModel.sdf</sdf>

      <author>
        <name>My name</name>
        <email>name@email.address</email>
      </author>

      <description>
        Description of MyModel
      </description>
    </model>

The example 2DOF manipulator model files are located *models* folder inside the root folder of oppt, since this folder is automatically added to the *OPPT_RESOURCE_PATH* environment variable when sourcing the setup.sh shell script inside the *oppt_root* folder. If you navigate to the *models/2DOFManipulator* folder you will find the 2DOFManipulator model file and the corresponding *model.config*. The *2DOFManipulator.sdf* contains a full kinematic and dynamic description of the robot. Now we also need an environment the robot operates in. Navigate to the *models/2DOFEnvironment* folder and you will find the environment SDF and its corresponding model.config file.

To add our 2DOF manipulator to the environment, we can use an include reference: 

    <include>
      <uri>model://2DOFManipulator</uri>
      <pose>0.0 0.0 0.0 0 0 0</pose>
    </include>
    
You will also find a description of the Gazebo physics engine. As of now, ODE is the only supported physics engine.
Another important component of a motion planning problem is the goal area the robot has to reach. The model of the goal area should have no *<collision>* tag, otherwise it is treated as an obstacle.

After we are finished defining our environment and robot model, we can provide a problem configuration file.

Note that while in this example we define the environment and robot models in separate SDF files, it is also possible to use a single SDF file which contains both the environment as well as the robot model.

### Problem configuration file

The problem configuration file contains all the configuration parameters to set-up a POMDP planning problem in its entirety, including state, action and observation descriptions, solver parameters, etc. Navigate to the *cfg/* folder and you will see a bunch of problem configuration files for the various problems that are shipped with OPPT. 

Let's have a look at the *2DOFManipulator.cfg* file, which is the problem configuration file for our 2DOFManipulator problem. A problem configuration file consists of *sections* and parameters for these section:

    [section]
    parameter1
    parameter2
    parameter2
    
The most important sections are the *plugins*, *state*, *action*, *observation* and *problem* sections. We will go through these sections using the 2DOFManipulator.cfg config file as an example. Looking at the 2DOFManipulator.cfg config file we see the following *plugins* section:

    [plugins]
    planningTerminalPlugin = libdefaultTerminalPlugin.so
    executionTerminalPlugin = libdefaultTerminalPlugin.so

    planningTransitionPlugin = libdefaultTransitionPlugin.so
    executionTransitionPlugin = libdefaultTransitionPlugin.so

    planningObservationPlugin = libdefaultObservationPlugin.so
    executionObservationPlugin = libdefaultObservationPlugin.so

    executionInitialBeliefPlugin = libdefaultInitialBeliefPlugin.so
    planningInitialBeliefPlugin = libdefaultInitialBeliefPlugin.so
    
    rewardPlugin = libdefaultRewardPlugin.so
    heuristicPlugin = libRRTHeuristicPlugin.so
    
Here we specify which shared library will be used as the respective POMDP plugin. Note that for the transition, observation, terminal and initial belief plugins we have included the same plugin twice (with a 'planning' and 'execution' prefix). This is due to the fact that OPPT distinguishes between the plugins used when planning a policy, and the ones that are used when executing a policy. Since we assume for the 2DOFManipulator problem that the planning model is the same as the actual model, we use the same plugin libraries for both planning and execution. However, for other problems it is possible to use different plugins for planning and execution.

Note that the shared libraries specified inside the *plugins* section must reside inside a subdirectory of a folder that has been added to the *OPPT_RESOURCE_PATH* environment variable, e.g. inside the subdirectories of the <oppt_root>/plugins folder.

Following the *plugins* section, we can see severeal plugin options sections. We will cover these sections later.
Next are the *state*, *action* and *observation* sections which specify the POMDP states, actions and observations:

    [state]
    jointPositions = [2DOFManipulator::joint1, 2DOFManipulator::joint2]
    jointVelocities = [2DOFManipulator::joint1, 2DOFManipulator::joint2]
    
With this description, OPPT models the states of the robot as a 4D-vector consisting of the joint angles and joint velocities. Note that variables that refer to different elements of the robot (i.e. joints and links) have to be in the scope of the robot name. Looking at the jointPosition variables above, joint1 and joint2 and in the scope of 2DOFManipulator, as indicated by the double colon. Next, we have to specify the POMDP actions: 

    [action]
    jointTorques = [2DOFManipulator::joint1, 2DOFManipulator::joint2]
    
This tells OPPT that the actions are 2D-vectors consisting of the joint torques. This specification also provides enough information to the GazeboInterface, such that an action vector is applied to the specified joints. Similarly, we have to specify the observations the robot can perceive:

    [observation]
    jointVelocities = [2DOFManipulator::joint1, 2DOFManipulator::joint2]
    linkPoses = [2DOFManipulator::link2]    
    
This specifies that the observations are 8D-vectors consisting of the joint velocities and the 3D-pose of the second link. The additional dimensions can then be handled inside the respective plugin (in this case the oppt::TransitionPlugin).

The state, action and observation descriptions not only tell OPPT how the spaces are structured, it also configures the oppt::GazeboInterface and the underlying physics engine. E.g. when stepping the physics engine forward in time using the oppt::GazeboInterface::doPropagation interface the torques specified in the *jointTorques* parameter will be applied to the respective joints.

After the state, action and observation descriptions you will see a *problem* section. The parameters in this section configure the actual POMDP planning problem. The most important parameters in this section are the *planningEnvironmentPath* and *executionEnvironmentPath* parameters containing the environment SDF files for planning and execution. Additionally, *robotName* specifies the name of the model in our SDF file which corresponds to the robot. Note that the physical entities of the robot used for the state, action and observation descriptions must be consistent with the SDF model of the robot specified by the *robotName* parameter.
For a full list of available options, see the oppt::ProblemEnvironmentOptions documentation.

Additionally the problem configuration file has to contain the parameters for a particular POMDP solver. For the 2DOFManipulator.cfg config file we have already defined the parameters for ABT under the *ABT* section. For custom POMDP solvers it is possible to define additional parameters as outlined in section @ref defCustomOptions.

# Defining a general POMDP problem {#general}
For problems that do not fall into the class of standard motion planning problems, such as grasping, target-tracking and environmental exploration problems, the user can 
provide custom implementations of the plug-ins that define a particular POMP problem. The plug-ins are designed such that only a small number of virtual methods have to be
implemented.

## POMDP plugins {#plugins}

In OPPT the components of a POMDP are implemented via plugins. Plugins are shared libraries that are loaded dynamically during runtime. There are 5 plugin types, each of them modelling a specific component of a POMDP model. These are:

 - oppt::TransitionPlugin 
 - oppt::ObservationPlugin
 - oppt::RewardPlugin
 - oppt::TerminalPlugin
 - oppt::InitialBeliefPlugin
 - oppt::HeuristicPlugin
 
Note that the state-, action- and observation space components of a POMDP model are automatically constructed during run-time, therefore these components don't require a plugin type.

### TransitionPlugin
In a POMDP model the transition function is a conditional probability distribution over the state space. It models the probability of ending up in state s' after taking action a from state s: P(s' | s, a). In OPPT the transition function is implicitly defined inside the oppt::TransitionPlugin. The oppt::TransitionPlugin::propagateState method takes as an input a pointer to oppt::PropagationRequest, which contains a state and an action, and returns a oppt::PropagationResult containing the next state. This can be seen as a black-box sampler of the actual transition probability distribution.

### ObservationPlugin
Similarly to the TransitionPlugin, the oppt::ObservationPlugin implicitly models the observation function which is a conditional probability distribution over the observation space, given a state s and and action a, such that P(o | s, a). The oppt::ObservationPlugin:getObservation takes as an input a oppt::ObservationRequest containing a state and an action outputs a oppt::ObservationResult containing the sampled observation.

### RewardPlugin
The oppt::RewardPlugin models the POMDP reward function, a function that maps states an actions to rewards. The oppt::RewardPlugin::getReward takes as an input a pointer to a oppt::PropagationResults, which containing a state, action, and next state, and outputs a reward.

### TerminalPlugin
In OPPT the oppt::TerminalPlugin serves two purposes: It checks wheter a state is a terminal state (via the oppt::TerminalPlugin::isValid method), or if a state is valid (via the oppt::TerminalPlugin::isTerminal method). In OPPT invalid states are always terminal but not the other way around (terminal states can still be valid)

### InitialBeliefPlugin
The oppt::InitialBeliefPlugin samples states according to an initial belief (via the oppt::InitialBeliefPlugin::sampleAnInitState method).

### Heuristic plugin
Many POMDP solvers - such as the solver that shipps with OPPT, ABT - require a function that calculates an estimation of the Q-value given a particular state: A heuristic function. For this purpose OPPT uses an oppt::HeuristicPlugin. Providing this plugin is optional. If one is provided, the solver has access to it via the *heuristicPlugin_* member.

## Implementing custom POMDP plugins {#implCustomPlugins}
Implementing custom plugins and using them for a specific problem model is simple. Here we will show an example implementation of a custom TransitionPlugin, which we will call MyTransitionPlugin.
Navigate to the src/plugins/transitionPlugins folder. Here you will see a few folders that contain specific implementations of TransitionPlugins that are used for the various problem examples that are shipped with OPPT. Let's create a new folder called "MyTransitionPlugin". Inside this folder, create a source file "MyTransitionPlugin.cpp" with the following contents:

    // Include the Plugins header file
    #include "oppt/plugin/Plugin.hpp"
    
    namespace oppt
    {
    class MyTransitionPlugin: public TransitionPlugin
    {
    public:
        MyTransitionPlugin():
            TransitionPlugin() {
        }
        
        virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override { 
            // This method is called after the plugin has been instantiated.
            // The first parameter is a pointer to the robot planning environment whose functionality can be used within the plugin
            // The second parameter is the path to the problem configuration file            
        }
        
        virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override { 
            // This method is called when a state has to be propagated via an action as specified in the propagationRequest
            // parameter
        }
    };
    
    // Here we need to register our plugin. This is done using the following preprocessor macro:
    OPPT_REGISTER_TRANSITION_PLUGIN(MyTransitionPlugin)
    
    } 
    
After we have provided an implementation of the oppt::TransitionPlugin, we have to build it as a shared library. In order to do this, create a new CMakeLists.txt inside the MyTransitionPlugin and add the ADD_TRANSITION_PLUGIN macro:

    ADD_TRANSITION_PLUGIN(myTransitionPlugin SHARED
                ${CMAKE_CURRENT_LIST_DIR}/MyTransitionPlugin.cpp)
                          
For the other plugin types, the user has access to the ADD_OBSERVATION_PLUGIN, ADD_INITIAL_BELIEF_PLUGIN, ADD_REWARD_PLUGIN, ADD_TERMINAL_PLUGIN and ADD_HEURISTIC_PLUGIN macros.
To build your TransitionPlugin, navigate to the src/build folder and type 'make'. After successful compilation, the shared library for the transition plugin will be located inside the
plugins/transitionPlugins folder. Now we can use our TransitionPlugin inside the problem configuration files:

    [plugins]
    planningTransitionPlugin = libmyTransitionPlugin.so
    executionTransitionPlugin = libmyTransitionPlugin.so
    
## Using general state, action and observation descriptions

In section @ref problemConfFile we have seen how to define states, actions and observations by relating physical quantities of the robot (such as joint angles and joint torques) to the specific POMDP state, action and observation descriptions. In many cases this is insufficient for a complete state description. For this OPPT provides an additional integer paramerter *additionalDimensions*, where the user can specifiy the number of dimensions. An example of a state description using the *additionalDimension* parameter can be found inside the *cfg/Rocksample.cfg* problem configuration file:

    [state]
    # The first state dimension is the x-position of the robot
    linkPositionsX = [RocksampleRobot::RocksampleLink]
    
    # The second state dimension is the y-position of the robot
    linkPositionsY = [RocksampleRobot::RocksampleLink]    

    # Rock states
    additionalDimensions = 8
    
With this the state of the robot is a 10-dimensional vector consisting of the xy-position of the robot and 8 additional dimensions describing the state of the environment.

# Implementing new solvers {#impl}
OPPT aims to ease the implementation of new POMP solvers. For this, OPPT provides a general solvers::Solver interface. To develop a new POMDP solver, the user needs to provide an implementation of the pure virtual methods solvers::Solver::setup, solvers::Solver::reset, solvers::Solver::improvePolicy, solvers::Solver::getNextAction and solvers::Solver::updateBelief. Inside these methods, a solvers::Solver has access to the oppt::RobotEnvironment (via the robotPlanningEnvironment_ member).

When a oppt::ProblemEnvironment executes a POMDP problem, it repeatedly calls solvers::Solver::improvePolicy, solvers::Solver::getNextAction and solvers::Solver::updateBelief (in this order), until a terminal condition is met, according to the oppt::TerminalPlugin. Note that OPPT does not enforce a specific belief data structure. Depending on the solver, a belief can have very different representations, such as a set of particles or a multivariate-normal distribution. An implementation of solvers::Solver therefore has to provide its own internal belief data structure.

To provide a solver implementation, navigate to the src/solvers folder and create a new folder for your solver. In this example we will simply call it MySolver. Inside the src/solvers/MySolver folder, create a MySolver.hpp header file which contains the implementation of your solver. This header file should look like this:

    #ifndef _MY_SOLVER_HPP_
    #define _MY_SOLVER_HPP_
    
    // Include the solver interface
    #include "solvers/solver.hpp"
    
    class MySolver: public solvers::Solver {
    public:
        MySolver():
          solver::Solver() {}          
        
        virtual void setup() override {
            // Perform your set-up operations here, such as initializing the initial belief
        }        
        
        virtual bool reset() override {
            // Here you should reset your internal datastructures 
        }
        
        virtual bool improvePolicy(const double& timeout) override {
            // Perform policy improvement starting from the current belief.
            // If the policy improvement was successful, return true,
            // otherwise return false
        }
        
        virtual ActionSharedPtr getNextAction() override {
            // Return the best action according to the policy calculated inside the improvePolicy() method.
            // If there is no action available, simply return a nullptr         
        }
        
        virtual bool updateBelief(const ActionSharedPtr& action, const ObservationSharedPtr& observation) override {
            // Update the current belief based on the action and the observation
            // and return true if the belief update was successful. Otherwise return false
        }
  
    };
    
    #endif

After we have provided a solver implementation, we need to provide a C++ main file that runs the oppt::ProblemEnvironment using MySolver as the POMDP solver. Let's create a new mainfile 'main_mysolver.cpp' inside the src/solvers/MySolver folder with the following contents:

    // Include the ProblemEnvironment
    #include "oppt/problemEnvironment/ProblemEnvironment.hpp"
    
    // Include our solver implementation
    #include "MySolver.hpp"
    
    // Include our solver options
    #include "MySolverOptions.hpp"
    
    int main(int argc, char const* argv[])
    {    
        // First we need an instance of ProblemEnvironment
        oppt::ProblemEnvironment problemEnvironment;
        
        // Next we need to call the setup template, where the first template parameter is 
        // our solver and the second template parameter is our solver options.
        int ret = problemEnvironment.setup<solvers::MySolver, oppt::MySolverOptions>(argc, argv);
        
        // If the return code of setup is anything else but 0, we shouldn't go on
        if (ret != 0)
            return ret;
            
        // Now we run the problem using our solver
        return problemEnvironment.runEnvironment(argc, argv);
    }
    
This main file will instantiate a oppt::ProblemEnvironment and set it up so that the POMDP solver being used is MySolver. Next it will execute the environment.

So far the src/solvers/MySolver folder should contain MySolver.hpp and main_mysolver.cpp (and a MySolverOptions.hpp header that will be discussed in @ref defCustomOptions). Now we need to create an executable for our solver. To do so we create a CMakeLists.txt inside the src/solvers/MySolver folder with the following contents:

    # Add a target for the solver
    add_executable(mysolver 
                   ${CMAKE_CURRENT_LIST_DIR}/main_mysolver.cpp
                   <additional source files>)
                   
    # Link the solver executable to the OPPT library
    target_link_libraries(mysolver
                          oppt
                          <additional libraries>)
                          
To build our solver, navigate to the src/build folder and type 'make'. Upon completion, you will find an executable 'mysolver' inside the bin folder. This executable can be used in 
a similar way as the already existing ABT solver:

    ./mysolver --cfg <ProblemConfiguationFile>
    
During execution the solver will automatically create a log file which contains information about the simulation runs. Per default, these log files are stored within the <oppt_root>/bin/log folder. If you want to store them in a different location, you have to change the 'logPath' option in your problem configuration file. 
    
# Defining custom options for solvers and plugins {#defCustomOptions}
Within a problem configuration file the user has a wide variety of available options to influence the behaviour of the oppt::ProblemEnvironment when running a solver for a specific problem. However, it is possible to define additional parameters used by a oppt::Solver or a custom plugin implementation if needed. This requires providing a oppt::Options struct that generates a parser for the user-defined options. Let's assume we have a problem configuration file where we need two additional parameters for our solver implementation of section @ref impl, a bool parameter and a string parameter. Navigate to the src/solver/MySolver folder and add a MySolverOptions.hpp file with the following contents:

    // Include the ProblemEnvironmentOptions data structure
    #include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"
    
    namespace oppt {
    
    // Our custom options data structure.
    // By inheriting from oppt::ProblemEnvironmentOptions, the resulting options object
    // will contain all members of the ProblemEnvironmentOptions data structure, plus
    // the additional members defined here
    struct MySolverOptions: public oppt::ProblemEnvironmentOptions {
        MySolverOptions() = default;
        virtual ~MySolverOptions() = default;
        
        // Our first custom parameter
        bool myBoolParameter;
        
        // Our second custom parameter
        std::string myStringParameter
        
        // Make a parser for our custom options datastructure
        static std::unique_ptr<options::OptionParser> makeParser(bool simulating) { 
            std::unique_ptr<options::OptionParser> parser =
                ProblemEnvironmentOptions::makeParser(simulating);
            addMyOptions(parser.get());  
            return std::move(parser);
        }
        
        static void addMyOptions(options::OptionParser* parser) {
            // Here we add our options to the parser using the addOption template. The
            // template parameter determines the type of our custom option
             
            // The first parameter is the section in the problem configuarion file
            // the option belongs to (can be empty). In this example both options
            // should be under MY_SECTION
            
            // The second parameter is the name of the custom option as used in the
            // problem configuration file.
            
            // The thid option is a reference to the MySolverOptions member this option belongs to
            parser->addOption<bool>("MY_SECTION", "myBoolParameter", &MySolverOptions::myBoolParameter);
            
            // We want the second option to have a default value
            parser->addOptionWithDefault<std::string>("MY_SECTION", "myStringParameter", &MHFROptions::myStringParameter, "defaultString");
        }
    };
    }
    
Going back to section @ref impl, we can see that we have already included our MySolverOptions header in the main_mysolver.cpp:

    #include "MySolverOptions.hpp"
    
Inside the main file, we set-up the oppt::ProblemEnvironment to use our custom options data structure:

    int ret = problemEnvironment.setup<solvers::MySolver, oppt::MySolverOptions>(argc, argv);
    
Now all we have to do is to add our options to the problem configuration file:

    [MY_SECTION]
    myBoolParameter = false
    myStringParameter = "myString"
    
And inside our MySolver implementation we have access to our custom options:

    static_cast<MySolverOptions*>(problemEnvironmentOptions_.get())->myBoolParameter;

Defining custom options for plugins works in a similar fashion. After we have defined a custom options struct for a particular plugin (see oppt::DefaultTransitionPluginOptions for an example), we can parse the options inside the load method of the plugin using 

    parseOptions_<MyPluginOptions>(optionsFile);

After this, we can use 

    static_cast<MyPluginOptions*>(options_.get())->optionsMember

to access our custom options.

# Working with normalized spaces {#workWNormalized}

OPPT provides an option to automatically normalize state-, action- and observation spaces such that each dimension of the spaces is normalized to the interval [0, 1]. This is useful for scaling the uncertainties a robot is subject to as well as simplifying theoretical analysis for solvers. When using normalized spaces, the state, action and observation limits have to be defined inside the problem configuration file. Example:

    [state]
    linkPositionsX = [link1, link2]
    
    # The limits of the x-position of the two links
    linkPositionXLimits = [[-10, 10], [-5, 5]]
    
Note that joint angle, velocitie and torque limits are deduced from the robot SDF model and don't have to be specifically defined inside the problem configuration file.

If required, normalized states, actions and observations can be de-normalized using the oppt::StateSpace::denormalizeState, oppt::ActionSpace::denormalizeAction and oppt::ObservationSpace::denormalizeObservation methods

# Using the playVideo executable {#playVideoExec}
As mentioned in @ref impl, running a solver on a specific problem produces a log file. These log files can be replayed using the *playVideo* executable inside the <oppt_root>/bin folder. Make sure that you have started the viewer (see section @ref quickStart) in a separate terminal window before replaying a log file:

    cd <oppt_root>/bin
    ./playVideo --cfg <ProblemConfigurationFile> --logfile <LogFile>
    
where the *cfg* parameter is the path to the problem configuration file and the *-f* parameter the path to the log file (both paths must be absolute).
For additional command line options, use 

    ./playVideo --help

--------------------------------------------------------------------------
OPPT Development Team
--------------------------------------------------------------------------

- Main developers:
    * Marcus Hoerger (OPPT toolkit)
    * Dimitri Klimenko (ABT solver)
    * Konstantin Seiler (continuous action spaces for ABT)     
