#ifndef __GENERATE_ABT_POLICY_HPP__
#define __GENERATE_ABT_POLICY_HPP__
#include "simulate.hpp"
#include "solvers/ABT/solver/Simulator.hpp"
#include "problems/robot_problem/changes/GenericModelChange.hpp"
#include "problems/robot_problem/treeInspector/TreeInspector.hpp"
#include "problems/robot_problem/abtSimulator/ABTSimulator.hpp"

namespace simulate
{

template<class RobotType>
class GenerateABTPolicy: public RobotSimulator
{
public:
    std::unique_ptr<robot::RobotModel> solverModel;
    std::unique_ptr<robot::RobotModel> simulatorModel;

    GenerateABTPolicy() :
        RobotSimulator() {
        this->algorithmName = "abt";
    }

    virtual ~GenerateABTPolicy() {

    }

    abt::ChangeSequence getChangeSequence(const std::string& change, const unsigned int& environmentChangeType) {
        // std::map<long, std::vector<std::unique_ptr<ModelChange>>> ChangeSequence;
        abt::ChangeSequence changeSequence;
        auto changePtr = std::make_unique<oppt::GenericModelChange>();
        changePtr->setChangeType(environmentChangeType);
        long nextStep = simulator_->getStepCount() + 1;
        changeSequence[nextStep].push_back(std::move(changePtr));
        return changeSequence;
    }

    virtual void handleEnvironmentChanges(const std::string& change, const unsigned int& environmentChangeType) override {
        if (simulator_) {
            // Set the change sequence
            simulator_->setChangeSequence(getChangeSequence(change, environmentChangeType));
        }
    }

    long getTreeDepth() {
        abt::BeliefNode* currentBelief = simulator_->getAgent()->getCurrentBelief();
    }

    int runSimulation(std::ofstream& os, int argc, char const* argv[]) override {
        oppt::PathPlannerSharedPtr path_planner = this->path_planner;
        oppt::RewardModelSharedPtr rewardModel = this->rewardModel_;
        std::shared_ptr<robot::RobotOptions> robot_options = this->robot_options;
        robot::RobotOptions options = *(this->robot_options.get());

        FloatType totalReward = 0;
        FloatType totalTime = 0;
        FloatType totalNSteps = 0;
        FloatType totalNHistories = 0;
        FloatType totalNSucRuns = 0;

        robot::resetRobotEnvironment<RobotType, robot::RobotOptions>(robotPlanningEnvironment, this->robot_options);
        robot::resetRobotEnvironment<RobotType, robot::RobotOptions>(robotExecutionEnvironment, this->robot_options);
        //RandomGenerator solverGen(this->randGen);
        this->randGen->discard(10000);
        if (!options.baseConfigPath.empty()) {
            oppt::change_directory(options.baseConfigPath);
        }

        solverModel =
            std::make_unique<robot::RobotModel>(this->randGen, std::make_unique<robot::RobotOptions>(options));
        solverModel->setPathPlanner(path_planner);        
        solverModel->setRobotEnvironment(robotPlanningEnvironment);        
        solverModel->setRewardModel(rewardModel);
        oppt::Solver solver(std::move(solverModel));
	solver.initializeEmpty();
        solver.improvePolicy();



        // Write the final policy to a file.
        cout << "Saving final policy..." << endl;
        std::ofstream outFile;
        std::ostringstream sstr;
        sstr << "final-ABT-policy.pol";
        outFile.open(sstr.str());
        if (!solver.getSerializer()) {
            ERROR("NO SERIALIZER");
        }
        solver.getSerializer()->save(outFile);
        outFile.close();
        cout << "Finished saving." << endl;


        return 0;
    }

private:
    std::unique_ptr<abt::Simulator> simulator_;
    //abt::Simulator* simulator_ = NULL;
};

}

#endif
