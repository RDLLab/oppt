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
#ifndef __PLAY_VIDEO_HPP__
#define __PLAY_VIDEO_HPP__
#include "oppt/opptCore/core.hpp"
#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "oppt/robotHeaders/RobotState.hpp"
#include <tclap/CmdLine.h>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unistd.h>
#include "solvers/solver.hpp"
#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace solvers
{
class SolverStub: public solvers::Solver
{
public:
    SolverStub():
        Solver() {

    }

    virtual void setup() override {};

    virtual bool reset() override {};

    virtual bool improvePolicy(const FloatType& timeout) override {};

    virtual ActionSharedPtr getNextAction() override {};

    virtual bool updateBelief(const ActionSharedPtr& action,
                              const ObservationSharedPtr& observation, const bool &allowTerminalStates=false) override {};

};
}

namespace oppt
{

struct SolverStubOptions: public ProblemEnvironmentOptions {
    SolverStubOptions() = default;
    virtual ~SolverStubOptions() = default;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(simulating);
        return std::move(parser);
    }
};

struct ParticleComponent {
    RobotStateSharedPtr particle;

    RobotStateSharedPtr previousParticle;
};

struct TimeSlice {
    RobotStateSharedPtr state;
    std::vector<ParticleComponent> particleComponents;
};

typedef std::vector<TimeSlice> TimeSlices;

struct LogFilePlayer {

    unsigned int particleSize = 100;

    std::vector<ParticleComponent> parseParticles(std::ifstream& is, const bool& finalState) {
        std::vector<ParticleComponent> particleComponents;
        std::string s = "";
        //std::getline(is, s);
        //unsigned int counter = 0;
        while (true) {
            VectorFloat stateValues;
            is >> s;
            if (s.find("PARTICLES") != std::string::npos) {
                break;
            }

            while (true) {
                is >> s;
                if (s.find("w:") != std::string::npos)
                    break;
                FloatType val;
                std::istringstream(s) >> val;
                stateValues.push_back(val);
            }


            ParticleComponent particleComponent;
            RobotStateSharedPtr particleState(new VectorState(stateValues));
            particleComponent.particle = particleState;

            // Consume weight
            is >> s;
            // Consume END
            is >> s;
            is >> s;
            if (s.find("USER_DATA_BEGIN") != std::string::npos) {
                VectorFloat previousParticleVec;
                while (true) {
                    is >> s;
                    if (s.find("w:") != std::string::npos)
                        break;
                    FloatType val;
                    std::istringstream(s) >> val;
                    previousParticleVec.push_back(val);
                }

                RobotStateSharedPtr previousParticle(new VectorState(previousParticleVec));
                particleComponent.previousParticle = previousParticle;
                std::getline(is, s);
            }

            particleComponents.push_back(particleComponent);
        }

        std::random_shuffle(particleComponents.begin(), particleComponents.end());
        std::vector<ParticleComponent> particlesReturn;
        for (size_t i = 0; i != particleSize; ++i) {
            particlesReturn.push_back(particleComponents[i]);
        }

        return particlesReturn;
    }

    TimeSlice parseFinalState(std::ifstream& is,
                              const ProblemEnvironment& problemEnvironment) {
        cout << "INSIDE FINAL" << endl;
        getchar();
    }

    TimeSlices parseStates(std::ifstream& is,
                           const ProblemEnvironment& problemEnvironment) {
        std::vector<RobotStateSharedPtr> robotStates;
        std::string s;
        bool breaking = false;
        bool finalState = false;
        TimeSlices timeSlices;
        while (!is.eof()) {
            std::vector<RobotStateSharedPtr> particles;
            while (true) {
                if (s.find("t =") != std::string::npos)
                    break;
                if (s.find("FINAL_STATE_BEGIN") != std::string::npos) {
                    finalState = true;
                    break;
                }
                if (s.find("PARTICLES BEGIN") != std::string::npos) {
                    timeSlices[timeSlices.size() - 1].particleComponents = parseParticles(is, finalState);
                    LOGGING("Parsed particles: " + std::to_string(timeSlices.size() - 1) + ": " + std::to_string(timeSlices[timeSlices.size() - 1].particleComponents.size()));
                }
                if (is.eof()) {
                    breaking = true;
                    break;
                }
                std::getline(is, s);
            }

            if (breaking)
                break;
            std::getline(is, s);
            std::istringstream sstr(s);
            auto serializer = problemEnvironment.getRobotExecutionEnvironment()->getRobot()->getSerializer();
            TimeSlice timeSlice;
            RobotStateSharedPtr robotState = serializer->loadState(sstr);
            timeSlice.state = robotState;
            timeSlices.push_back(timeSlice);
            std::getline(is, s);
        }

        return timeSlices;
    }

    void grabScreenshot(ros::Publisher& grabScreenPublisher,
                        const std::string& screenshotPath,
                        const unsigned int& num) {
        std::ostringstream ostr;
        ostr << std::setfill('0') << std::setw(5) << (num < 0 ? -num : num);
        std::string numString = ostr.str();
        std_msgs::String msg;
        std::string filename = screenshotPath + "/screenshot_" + numString + ".png";
        cout << "save to file: " << filename << endl;
        msg.data = filename;
        grabScreenPublisher.publish(msg);
        ros::spinOnce();
    }

    void playTimeSlices(const TimeSlices& timeSlices,
                        const ProblemEnvironment& problemEnvironment,
                        const std::string& screenshotPath,
                        const unsigned int& sleepTime,
                        const bool& userInput) {
        int argc = 0;
        char** argv;
        ros::init(argc, argv, "PlayVideo", ros::init_options::AnonymousName);
        if (ros::master::check()) {
            ros::NodeHandle node;
            ros::Publisher grabScreenPublisher =
                node.advertise<std_msgs::String>("grabScreenTopic", 0, true);
            unsigned int counter = 0;
            auto robot = problemEnvironment.getRobotExecutionEnvironment()->getRobot();

            //cout << "displaying text" << endl;
            //robot->getViewer()->as<ViewerPublisher>()->displayText("helloooo");
            //getchar();
	    VectorFloat particleColor = VectorFloat({0.2, 1.0, 0.2, 0.3});
            for (size_t i = 0; i != timeSlices.size(); i++) {
                FloatType t = 0;
                FloatType tDelta = (1.0 / (FloatType)(timeSlices[i].state->getSubStates().size()));
                unsigned int counter2 = 0;
                for (auto & subState : timeSlices[i].state->getSubStates()) {
                    std::vector<RobotStateSharedPtr> particles;
                    if (i > 0 && timeSlices[i - 1].particleComponents.size() == particleSize) {
                        for (size_t j = 0; j != particleSize; ++j) {
                            RobotStateSharedPtr interpolatedState = robot->getStateSpace()->interpolate(timeSlices[i - 1].particleComponents[j].previousParticle,
                                                                    timeSlices[i - 1].particleComponents[j].particle,
                                                                    t);
                            particles.push_back(interpolatedState);
                        }
                    }

                    robot->updateViewer(subState, particles, particleColor);
                    counter++;
                    t += tDelta;
                    usleep(((FloatType)sleepTime / 1000.0) * 1e6);
                    if (!screenshotPath.empty())
                        grabScreenshot(grabScreenPublisher, screenshotPath, counter);
                    counter2++;
                    if (counter2 == timeSlices[i].state->getSubStates().size() - 1) {
                        break;
                    }
                }

                cout << "UPDATE STATE " << i << endl;

                std::vector<RobotStateSharedPtr> finalParticles;
                if (i > 0) {
                    for (size_t j = 0; j != timeSlices[i - 1].particleComponents.size(); ++j) {
                        finalParticles.push_back(timeSlices[i - 1].particleComponents[j].particle);
                    }
                }

                robot->updateViewer(timeSlices[i].state, finalParticles, particleColor);
                counter++;
                usleep(((FloatType)sleepTime / 1000.0) * 1e6);
                if (!screenshotPath.empty())
                    grabScreenshot(grabScreenPublisher, screenshotPath, counter);
                if (userInput) {
                    char in = getchar();
                    if (in == 's') {
                        LOGGING("Skipping next step");
                        i += 1;
                    }
                }
            }
        }
    }
    
    int play(int argc, char const* argv[]) {

        //std::string argvStr(std::string::c_str(argv[0]));
        TCLAP::CmdLine cmd("Command description message", ' ', "0.9");
        TCLAP::ValueArg<std::string> logfileArg("f", "logfile", "Path to the logfile", true, "", "string");
        TCLAP::ValueArg<std::string> configFileArg("c", "cfg", "Path to the config file", true, "", "string");
        TCLAP::ValueArg<std::string> screenshotPathArg("s", "screenshotPath", "Path to where the screenshots are saved", false, "", "string");
        TCLAP::ValueArg<unsigned int> sleepTimeArg("z", "sleepTime", "Sleep time between steps (in ms)", true, 100, "unsigned int");
        TCLAP::ValueArg<unsigned int> particleNumArg("p", "particles", "Number of rendered particles", true, 10, "unsigned int");

        TCLAP::SwitchArg userInputArg("u", "userInput", "Wait for user input", false);

        cmd.add(logfileArg);
        cmd.add(configFileArg);
        cmd.add(screenshotPathArg);
        cmd.add(particleNumArg);
        cmd.add(sleepTimeArg);
        cmd.add(userInputArg);
        cmd.parse(argc, argv);

        std::string logFile = logfileArg.getValue();
        std::string configFile = configFileArg.getValue();
        std::string screenshotPath = screenshotPathArg.getValue();
        unsigned int particleNum = particleNumArg.getValue();
        unsigned int sleepTime = sleepTimeArg.getValue();
        bool userInput = userInputArg.getValue();

        std::ifstream is(logFile);
        if (!is.good()) {
            cout << "Error: File " << logFile << " doesn't exist" << endl;
            return 2;
        }

        std::string exeString = "./playVideo";
        std::string cfgString = "--cfg";
        int argc2 = 3;
        const char* argv2[] = {exeString.c_str(), cfgString.c_str(), configFile.c_str()};

        ProblemEnvironment problemEnvironment;
        int ret = problemEnvironment.setup<solvers::SolverStub, SolverStubOptions>(argc2, argv2);
        particleSize = particleNum;

        TimeSlices timeSlices = parseStates(is, problemEnvironment);
        playTimeSlices(timeSlices, problemEnvironment, screenshotPath, sleepTime, userInput);
        return 0;
    }

};



}

#endif
