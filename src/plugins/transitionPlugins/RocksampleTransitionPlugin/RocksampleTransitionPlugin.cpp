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
#include "oppt/opptCore/Distribution.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

namespace oppt
{
class RocksampleTransitionPlugin: public TransitionPlugin
{
public:
    typedef std::function<bool()> DubinCollisionFunction;

    RocksampleTransitionPlugin():
        TransitionPlugin(),
        rockLocations_() {
    }

    virtual ~RocksampleTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // Get the rock locations from the robotEnvironment
        getRockLocations_();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();

        // Copy the current state vector into the resulting state vector
        VectorFloat resultingState(propagationRequest->currentState->as<VectorState>()->asVector());
        switch ((unsigned int)(actionVec[0] + 0.025)) {
        case 1: {
            resultingState[1] = (FloatType)((unsigned int)(resultingState[1] + 0.25) + 1); // Go north
            // Ensure that the rover remains inside the map
            if (resultingState[1] > 7.5)
                resultingState[1] = 7.0;
            break;
        } case 2: {
            resultingState[1] = (FloatType)((unsigned int)(resultingState[1] + 0.25) - 1); // Go south
            // Ensure that the rover remains inside the map
            if (resultingState[1] < 0.5)
                resultingState[1] = 1.0;
            break;
        } case 3: {
            resultingState[0] = (FloatType)((unsigned int)(resultingState[0] + 0.25) + 1); // Go east
            break;
        } case 4: {
            resultingState[0] = (FloatType)((unsigned int)(resultingState[0] + 0.25) - 1); // Go west
            // Ensure that the rover remains inside the map
            if (resultingState[0] < 0.5)
                resultingState[0] = 1.0;
            break;
        } case 5: {
            sampleRockAtCurrentLocation_(resultingState); // Sample rock at the current location
            break;
        }
        default:  // We perform a checkRock action. For these actions, the state remains the same
            break;
        }

        propagationResult->nextState =
            std::make_shared<oppt::VectorState>(resultingState);

        // The following lines set the state of the robot in the Gazebo environment. This is needed for visualization.
        if (robotEnvironment_->isExecutionEnvironment()) {
            robotEnvironment_->getGazeboInterface()->setStateVector(resultingState);
            auto newWorldState = robotEnvironment_->getGazeboInterface()->getWorldState(true);
            propagationResult->nextState->setGazeboWorldState(newWorldState);
        }
        return propagationResult;
    }

private:
    // Rock indices and their respective location
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

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

    void sampleRockAtCurrentLocation_(VectorFloat& stateVec) const {
        // Check if there's a rock at the current rover location
        unsigned int roverLocationX = (unsigned int)(stateVec[0] + 0.25);
        unsigned int roverLocationY = (unsigned int)(stateVec[1] + 0.25);
        for (auto & rockLocation : rockLocations_) {
            if (roverLocationX == (unsigned int)(rockLocation.second[0] + 0.25) and
                    roverLocationY == (unsigned int)(rockLocation.second[1] + 0.25)) {
                // We found a rock at the current rover location and sampled it. Subsequently, the rock turns bad (= 0)
                stateVec[rockLocation.first] = 0;
                return;
            }
        }
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(RocksampleTransitionPlugin)

}
