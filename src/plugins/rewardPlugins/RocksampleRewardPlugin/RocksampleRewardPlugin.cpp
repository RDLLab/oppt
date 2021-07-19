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
#include "RocksampleRewardOptions.hpp"

namespace oppt
{
class RocksampleRewardPlugin: public RewardPlugin
{
public:
    RocksampleRewardPlugin():
        RewardPlugin() {

    }

    virtual ~RocksampleRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<RocksampleRewardOptions>(optionsFile);
        getRockLocations_();

        exitReward_ = static_cast<const RocksampleRewardOptions *>(options_.get())->exitReward;
        goodRockSamplingReward_ = static_cast<const RocksampleRewardOptions *>(options_.get())->goodRockSamplingReward;
        badRockSamplingPenalty_ = static_cast<const RocksampleRewardOptions *>(options_.get())->badRockSamplingPenalty;
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        VectorFloat previousStateVector = propagationResult->previousState->as<VectorState>()->asVector();
        VectorFloat currentStateVector = propagationResult->nextState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

        // If we moved out to the goal area at the right side of the map, return reward 10
        if (currentStateVector[0] > 7.5)
            return exitReward_;

        switch ((unsigned int)(actionVec[0] + 0.25)) {
        case 5: {
            // We performed a sampling action. Return the corresponding reward/penalty
            return getRockSamplingReward_(previousStateVector);            
        } default:            
            break;
        }

        // For all other actions, we receive a reward of 0.0
        return 0.0;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-10.0, 10.0);
    }

private:
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

    FloatType exitReward_ = 0.0;

    FloatType goodRockSamplingReward_ = 0.0;

    FloatType badRockSamplingPenalty_ = 0.0;

private:
    void getRockLocations_() {
        auto scene = robotEnvironment_->getScene();
        VectorBodyPtr bodies = scene->getBodies();
        unsigned int idx = 2;
        for (auto & body : bodies) {
            if (body->getOpptCollisionObjects().size() != 0) {
                auto pose = body->getWorldPose();
                VectorFloat location(2, 0);
                location[0] = pose.position[0];
                location[1] = pose.position[1];
                rockLocations_[idx] = location;
                idx++;
            }
        }
    }

    FloatType getRockSamplingReward_(const VectorFloat& previousStateVec) const {
        unsigned int roverLocationX = (unsigned int)(previousStateVec[0] + 0.25);
        unsigned int roverLocationY = (unsigned int)(previousStateVec[1] + 0.25);

        // First, find the rock at the rover location (if there's one)
        for (auto & rockLocation : rockLocations_) {
            if (roverLocationX == (unsigned int)(rockLocation.second[0] + 0.25) and
                    roverLocationY == (unsigned int)(rockLocation.second[1] + 0.25)) {
                // We found a rock at the rover location
                FloatType reward = 0;
                if ((unsigned int)(previousStateVec[rockLocation.first] + 0.25) == 1) {
                    // The rock was good. So the robot gets a reward of goodRockSamplingReward_ for sampling it
                    reward = goodRockSamplingReward_;
                } else if ((unsigned int)(previousStateVec[rockLocation.first] + 0.25) == 0) {
                    // The rock was bad. So the robot gets a penalty of badRockSamplingPenalty_ for sampling it
                    reward = badRockSamplingPenalty_;
                } else {
                    ERROR("Good bad rock turned good by sampling?!");
                }

                return reward;
            }
        }

        // We attempted to sample a rock at the rover location, but there is no rock.
        // For this we receive a penalty of badRockSamplingPenalty_
        return badRockSamplingPenalty_;
    }

};

OPPT_REGISTER_REWARD_PLUGIN(RocksampleRewardPlugin)
}
