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
        VectorFloat initStateVec = options->initialStateVec;
        if (initStateVec.size() != robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions())
            ERROR("Init state size doesnt fit");
        unsigned int stateDimension = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();
        std::uniform_int_distribution<unsigned int> d(0, 1);
        auto randomGenerator = robotEnvironment_->getRobot()->getRandomEngine();

        // Rock are good/bad with equal probability
        for (size_t i = 2; i != stateDimension; ++i) {
            initStateVec[i] = (FloatType)d(*(randomGenerator.get()));
        }

        RobotStateSharedPtr initState(new VectorState(initStateVec));
        return initState;
    }
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(RocksampleInitialBeliefPlugin)

}

#endif
