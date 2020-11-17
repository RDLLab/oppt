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
        getRockLocations();
        return true;
    }

    FloatType sampleRockAtCurrentLocation(const VectorFloat& previousStateVec,
                                          const VectorFloat& currentStateVec, const VectorFloat& actionVec) const {
        // Check if there's a rock at the current location
        unsigned int pS1 = (unsigned int)(previousStateVec[0] + 0.25);
        unsigned int pS2 = (unsigned int)(previousStateVec[1] + 0.25);
        unsigned int s1 = (unsigned int)(currentStateVec[0] + 0.25);
        unsigned int s2 = (unsigned int)(currentStateVec[1] + 0.25);
        for (auto & rockLocation : rockLocations_) {
            if (s1 == (unsigned int)(rockLocation.second[0] + 0.25) && s2 == (unsigned int)(rockLocation.second[1] + 0.25)) {
                FloatType reward = 0;
                if ((unsigned int)(previousStateVec[rockLocation.first] + 0.25) == 1 &&
                        (unsigned int)(currentStateVec[rockLocation.first] + 0.25) == 0) {
		    reward = 10;
                } else if ((unsigned int)(previousStateVec[rockLocation.first] + 0.25) == 0 &&
                           (unsigned int)(currentStateVec[rockLocation.first] + 0.25) == 0) {
		    reward = -10.0;
                } else {
                    ERROR("Good bad rock turned good by sampling?!");
                }
                
                return reward;
            }
        }

        return -10.0;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {       
        VectorFloat previousStateVector = propagationResult->previousState->as<VectorState>()->asVector();
        VectorFloat currentStateVector = propagationResult->nextState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

        // If we moved out of the right side of the map, return reward 10
        if (currentStateVector[0] > 7.5) {
            return 10.0;
        } else if (currentStateVector[0] < 0.5 || currentStateVector[0] > 8.5 || currentStateVector[1] < 0.5 || currentStateVector[1] > 7.5) {
            // Illegal move => return penalty
            return -10.0;
        }

        switch ((unsigned int)(actionVec[0] + 0.25)) {
        case 5: return sampleRockAtCurrentLocation(previousStateVector, currentStateVector, actionVec);
            break;
        default: return 0.0;
            break;
        }
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-10.0, 10.0);
    }

private:
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

private:
    void getRockLocations() {
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

};

OPPT_REGISTER_REWARD_PLUGIN(RocksampleRewardPlugin)
}
