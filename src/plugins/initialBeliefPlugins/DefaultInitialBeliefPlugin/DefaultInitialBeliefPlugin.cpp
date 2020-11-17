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
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "DefaultInitialBeliefOptions.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "oppt/opptCore/Distribution.hpp"

namespace oppt
{
class DefaultInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    DefaultInitialBeliefPlugin():
        InitialBeliefPlugin(),
        initialStateVec_() {
    }

    virtual ~DefaultInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {        
        parseOptions_<DefaultInitialBeliefOptions>(optionsFile);
        auto options = static_cast<DefaultInitialBeliefOptions*>(options_.get());
        unsigned int numDimensions = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();
        if (options->lowerBound.size() != numDimensions)
            ERROR("Lower bound for the uniform distribution doesn't match state space dimension");
        if (options->upperBound.size() != numDimensions)
            ERROR("Upper bound for the uniform distribution doesn't match state space dimension");
        for (size_t i = 0; i != options->lowerBound.size(); ++i) {
            if (options->lowerBound[i] > options->upperBound[i])
                ERROR("Lower bound for initial belief must be smaller than upper bound");
        }
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        uniformDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(options->lowerBound, options->upperBound, randomEngine);
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        // Sample an initial state vector
        VectorFloat initialStateVec = toStdVec<FloatType>(uniformDistribution_->sample(1).col(0));

        // First construct an intial world state based in the intial state vector
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVec);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState_ = robotEnvironment_->getGazeboInterface()->getInitialWorldState();


        RobotStateSharedPtr initialState = std::make_shared<VectorState>(initialStateVec);
        initialState->setGazeboWorldState(initialWorldState_);
        return initialState;
    }

private:
    VectorFloat initialStateVec_;

    std::unique_ptr<Distribution<FloatType>> uniformDistribution_;

};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(DefaultInitialBeliefPlugin)

}

#endif
