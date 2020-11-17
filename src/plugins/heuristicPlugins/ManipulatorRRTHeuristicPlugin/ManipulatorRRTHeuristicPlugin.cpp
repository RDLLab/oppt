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
#include "oppt/plugin/Plugin.hpp"
#include "plugins/heuristicPlugins/CarMazeRRTHeuristicPlugin/RRTHeuristicOptions.hpp"
#include "oppt/opptCore/HeuristicInfo.hpp"
#include "plugins/heuristicPlugins/shared/RRTConnect/RRTConnect.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

namespace oppt
{
class ManipulatorRRTHeuristicPlugin: public HeuristicPlugin
{
public:
    ManipulatorRRTHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~ManipulatorRRTHeuristicPlugin() {

    }

    virtual bool load(const std::string& optionsFile) override {
        optionsFile_ = optionsFile;        
        parseOptions_<RRTHeuristicOptions>(optionsFile);        

        rrtConnect_ =
            std::make_unique<RRTConnect>(robotEnvironment_,
                                         static_cast<RRTHeuristicOptions*>(options_.get())->planningRange);
        // Get the goal states
        std::vector<VectorFloat> goalStatesVec;
        auto goalStates = loadGoalStates();
        for (auto & goalState : goalStates) {
            goalStatesVec.push_back(goalState->as<VectorState>()->asVector());
        }

        rrtConnect_->setGoalStates(goalStatesVec);

        // Set the distance function
        unsigned int numStateDimensions = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();        
        DistanceFunction d = [numStateDimensions](const RobotStateSharedPtr &s1, const RobotStateSharedPtr &s2){
            VectorFloat s1Vec = s1->as<VectorState>()->asVector();
            VectorFloat s2Vec = s2->as<VectorState>()->asVector();
            return math::euclideanDistance(s1Vec.data(), s2Vec.data(), numStateDimensions / 2);            
        };

        rrtConnect_->setDistanceFunction(d);
        
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo * heuristicInfo) const override {
        return getHeuristicValueImpl(heuristicInfo);
    }
    
private:    
    std::unique_ptr<RRTConnect> rrtConnect_;
    std::string optionsFile_ = "";

private:
    VectorRobotStatePtr loadGoalStates() const {
        // TODO: Support more than one goal state
        auto options = static_cast<RRTHeuristicOptions*>(options_.get());
        VectorRobotStatePtr goalStates(1);
        goalStates[0] = std::make_shared<VectorState>(options->goalState);
        VectorRobotStatePtr normalizedGoalStates(goalStates.size());
        for (size_t i = 0; i < goalStates.size(); i++) {
            VectorFloat goalStateVec = goalStates[i]->as<VectorState>()->asVector();
            if (goalStateVec.size() != robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions()) {
                ERROR("The loaded goal states have a different size that the state space: " +
                      std::to_string(goalStateVec.size()) + " vs " +
                      std::to_string(robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions()));
            }

            //goalStates[i]->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getInitialWorldState());

            //PropagationResultSharedPtr propRes(new PropagationResult);
            //propRes->nextState = goalStates[i];
            RobotStateSharedPtr normalizedGoalState =
                robotEnvironment_->getRobot()->getStateSpace()->normalizeState(goalStates[i]);
            normalizedGoalStates[i] = normalizedGoalState;
        }

        return normalizedGoalStates;
    }

    FloatType getHeuristicValueImpl(const HeuristicInfo * heuristicInfo) const {
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        propagationResult->nextState = heuristicInfo->currentState;
        if (!(robotEnvironment_->isValid(propagationResult))) {
            //LOGGING("NOT VALID");
            return 0.0;
        }
        TrajectorySharedPtr trajectory =
            rrtConnect_->solve(heuristicInfo->currentState, heuristicInfo->timeout);
        if (!trajectory) {
            //LOGGING("NO TRAJECTORY");
            return 0;
        }

        return getTrajectoryValue_(trajectory, heuristicInfo);
    }



    FloatType getTrajectoryValue_(const TrajectorySharedPtr & trajectory,
                                  const HeuristicInfo * heuristicInfo) const {
        FloatType reward = 0.0;
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        auto minMaxReward = robotEnvironment_->getRewardPlugin()->getMinMaxReward();
        VectorRobotStatePtr traj;
        for (size_t i = 0; i != trajectory->stateTrajectory.size(); ++i) {
            traj.push_back(trajectory->stateTrajectory[i]);
            propagationResult->nextState = trajectory->stateTrajectory[i];
            //cout << "traj state: " << *(propagationResult->nextState.get()) << endl;
            FloatType rewardState = robotEnvironment_->getReward(propagationResult);
            //FloatType rewardState = heuristicInfo->rewardPlugin->getReward(propagationResult.get());
            reward +=
                std::pow(heuristicInfo->discountFactor, heuristicInfo->currentStep + i) *
                rewardState;

            if (rewardState == minMaxReward.second)
                break;
        }

        return reward;
    }
};

OPPT_REGISTER_HEURISTIC_PLUGIN(ManipulatorRRTHeuristicPlugin)

}
