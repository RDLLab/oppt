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

namespace oppt
{
class RocksampleHeuristicPlugin: public HeuristicPlugin
{
public:
    RocksampleHeuristicPlugin():
        HeuristicPlugin(),
        rockLocations_() {

    }

    virtual ~RocksampleHeuristicPlugin() {

    }

    virtual bool load(const std::string& optionsFile) override {
        optionsFile_ = optionsFile;
        getRockLocations();
        return true;
    }

    FloatType euclideanDist(const VectorFloat& stateVec, const VectorFloat& rockLocation) const {
        return std::sqrt(std::fabs(stateVec[0] - rockLocation[0]) +
                         std::fabs(stateVec[1] - rockLocation[1]));
        VectorFloat robotPosition( {stateVec[0], stateVec[1]});
        return math::euclideanDistance<FloatType>(robotPosition, rockLocation);
    }

    FloatType manhattanDist(const VectorFloat& p1, const VectorFloat& p2) const {
        FloatType val = std::fabs(p2[0] - p1[0]);
        val += std::fabs(p2[1] - p1[1]);
        return val;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {        
        return getHeuristicValueAdvanced(heuristicInfo);
    }

    FloatType getHeuristicValueAdvanced(const HeuristicInfo* heuristicInfo) const {
	auto stateSpace = robotEnvironment_->getRobot()->getStateSpace();
        VectorFloat stateVec =
            heuristicInfo->currentState->as<VectorState>()->asVector();
        FloatType currentDiscount = std::pow(heuristicInfo->discountFactor, heuristicInfo->currentStep);        
        FloatType val = 0;
	
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
            FloatType lowestDist = manhattanDist(currendPos, rockLocations_.at(*it));
            ++it;
            for (; it != goodRocks.end(); ++it) {
                auto dist = manhattanDist(currendPos, rockLocations_.at(*it));
                if (dist < lowestDist) {
                    bestRock = *it;
                    lowestDist = dist;
                }
            }
            
            currentDiscount *= std::pow(heuristicInfo->discountFactor, (unsigned int)(lowestDist + 0.25));
            val += currentDiscount * 10.0;            
            goodRocks.erase(bestRock);
            currendPos = rockLocations_.at(bestRock);
        }

        // Move to the goal area
        VectorFloat gVec( {currendPos[0], 8.0});
        FloatType distToGoal = manhattanDist(currendPos, gVec);
        currentDiscount *= std::pow(heuristicInfo->discountFactor, (unsigned int)(distToGoal + 0.25));
        val += currentDiscount * 10.0;
	return val;
    }

    FloatType getHeuristicValueSimple(const HeuristicInfo* heuristicInfo) const {
	auto stateSpace = robotEnvironment_->getRobot()->getStateSpace();
        VectorFloat stateVec =
            heuristicInfo->currentState->as<VectorState>()->asVector();
        FloatType val = 8.0 - stateVec[0];
        if (val < 0.5)
            return 10.0;
        FloatType heuristic = 1.0 / (FloatType)val;
        return heuristic;
    }   

private:
    std::string optionsFile_;

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

OPPT_REGISTER_HEURISTIC_PLUGIN(RocksampleHeuristicPlugin)

}

#endif
