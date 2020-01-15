#ifndef _SHOW_SCENE_HPP_
#define _SHOW_SCENE_HPP_
#include "oppt/opptCore/core.hpp"
#include <tclap/CmdLine.h>
#include "solvers/solver.hpp"
#include "oppt/problemEnvironment/ProblemEnvironment.hpp"

namespace oppt
{

class ShowSceneSolverStub: public solvers::Solver
{
public:
    ShowSceneSolverStub():
        Solver() {

    }

    virtual void setup() override {};

    virtual bool reset() override { return true; };

    virtual bool improvePolicy(const FloatType& timeout) override { return false;};

    virtual ActionSharedPtr getNextAction() override {return nullptr;};

    virtual bool updateBelief(const ActionSharedPtr& action,
                              const ObservationSharedPtr& observation) override { return false;};

};

struct ShowSceneSolverStubOptions: public ProblemEnvironmentOptions {
    ShowSceneSolverStubOptions() = default;
    virtual ~ShowSceneSolverStubOptions() = default;

    static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
        std::unique_ptr<options::OptionParser> parser =
            ProblemEnvironmentOptions::makeParser(simulating);
        return std::move(parser);
    }
};

class ShowScene
{
public:
    ShowScene() = default;
    ~ShowScene() = default;

    int show(int argc, char const* argv[]) const
    {
        TCLAP::CmdLine cmd("Command description message", ' ', "0.9");
        TCLAP::ValueArg<std::string> configFileArg("c", "cfg", "Path to the config file", true, "", "string");
        cmd.add(configFileArg);
        cmd.parse(argc, argv);
        std::string configFile = configFileArg.getValue();
        cout << configFile << endl;

        std::string exeString = "./showScene";
        std::string cfgString = "--cfg";
        int argc2 = 3;
        const char* argv2[] = {exeString.c_str(), cfgString.c_str(), configFile.c_str()};

        ProblemEnvironment problemEnvironment;
        int ret = problemEnvironment.setup<ShowSceneSolverStub, ShowSceneSolverStubOptions>(argc2, argv2);
        int ret2 = problemEnvironment.runEnvironment(argc2, argv2);
	cout << "Press enter to exit" << endl;
	getchar();
    }

};
}

#endif
