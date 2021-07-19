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
#ifndef _ROCKAMPLE_HEURISTIC_PLUGIN_HPP_
#define _ROCKAMPLE_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "RocksampleHeuristicOptions.hpp"

namespace oppt
{
class RocksampleHeuristicPlugin: public HeuristicPlugin
{
public:
    RocksampleHeuristicPlugin():
        HeuristicPlugin(),
        rockLocations_() {

    }

    virtual ~RocksampleHeuristicPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // First, parse the optionsFile into a RocksampleHeuristicOptions object
        parseOptions_<RocksampleHeuristicOptions>(optionsFile);
        getRockLocations();

        // Set the exitReward_ and goodRockSamplingReward_ to the values specified by the options
        exitReward_ = static_cast<const RocksampleHeuristicOptions *>(options_.get())->exitReward;
        goodRockSamplingReward_ = static_cast<const RocksampleHeuristicOptions *>(options_.get())->goodRockSamplingReward;
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        // To compute a heuristic estimate of the Q-value, we use a simple greddy policy 
        // that visits and samples all rocks that are good according to the state 
        // provided in heuristicInfo->currentState and then moves to the goal area        
        VectorFloat stateVec =
            heuristicInfo->currentState->as<VectorState>()->asVector();
        FloatType currentDiscount = 1.0;
        FloatType totalDiscountedReward = 0;        

        // Collect all the good rocks
        std::set<unsigned int> goodRocks;
        for (size_t i = 2; i != stateVec.size(); ++i) {
            if (stateVec[i] > 0.5) {
                goodRocks.insert(i);
            }
        }

        // Visit the rocks in greedy order
        VectorFloat currendPos( {stateVec[0], stateVec[1]});
        while (!goodRocks.empty()) {
            std::set<unsigned int>::iterator it = goodRocks.begin();
            unsigned int bestRock = *it;
            FloatType lowestDist = manhattanDist_(currendPos, rockLocations_.at(*it));
            ++it;
            for (; it != goodRocks.end(); ++it) {
                auto dist = manhattanDist_(currendPos, rockLocations_.at(*it));
                if (dist < lowestDist) {
                    bestRock = *it;
                    lowestDist = dist;
                }
            }

            currentDiscount *= std::pow(heuristicInfo->discountFactor, lowestDist);
            totalDiscountedReward += currentDiscount * goodRockSamplingReward_;
            goodRocks.erase(bestRock);
            currendPos = rockLocations_.at(bestRock);
        }


        // Move to the goal area
        VectorFloat gVec( {currendPos[0], 8.0});
        FloatType distToGoal = 8.0 - currendPos[0];
        currentDiscount *= std::pow(heuristicInfo->discountFactor, distToGoal);
        totalDiscountedReward += currentDiscount * exitReward_;
        return totalDiscountedReward;
    }    

private:
    // The reward of entering the goal area
    FloatType exitReward_ = 0.0;

    // The reward of sampling a good rock
    FloatType goodRockSamplingReward_ = 0.0;

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

    FloatType manhattanDist_(const VectorFloat& p1, const VectorFloat& p2) const {
        FloatType dist = std::fabs(p2[0] - p1[0]);
        dist += std::fabs(p2[1] - p1[1]);
        return dist;
    }

};

OPPT_REGISTER_HEURISTIC_PLUGIN(RocksampleHeuristicPlugin)

}

#endif
