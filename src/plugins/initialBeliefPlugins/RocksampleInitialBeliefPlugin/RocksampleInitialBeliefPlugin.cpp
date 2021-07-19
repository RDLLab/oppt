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
#ifndef _ROCKSAMPLE_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _ROCKSAMPLE_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "RocksampleInitialBeliefOptions.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

namespace oppt
{
class RocksampleInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    RocksampleInitialBeliefPlugin():
        InitialBeliefPlugin() {

    }

    virtual ~RocksampleInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<RocksampleInitialBeliefOptions>(optionsFile);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        auto options = static_cast<RocksampleInitialBeliefOptions*>(options_.get());

        // Construct an 10-dimensional state vector              
        VectorFloat initialStateVector(robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions(), 0.0);

        // The first two dimensions of the initial state vector are the rover position as specified in the configuration file
        initialStateVector[0] = options->initialRoverPosition[0];
        initialStateVector[1] = options->initialRoverPosition[1];

        // The next 8 dimensions of the initial state are the rock states. We set them to be bad (= 0) 
        // or good (= 1) with equal probability
        std::uniform_int_distribution<unsigned int> d(0, 1);
        auto randomGenerator = robotEnvironment_->getRobot()->getRandomEngine();        
        for (size_t i = 2; i != robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions(); ++i) {
            initialStateVector[i] = (FloatType)d(*(randomGenerator.get()));
        }

        // Construct the initial state from the initialStateVector
        RobotStateSharedPtr initialState(new VectorState(initialStateVector));
        if (robotEnvironment_->isExecutionEnvironment()) {
            // Required for visualization
            initialState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getInitialWorldState());
        }
                
        return initialState;
    }
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(RocksampleInitialBeliefPlugin)

}

#endif
