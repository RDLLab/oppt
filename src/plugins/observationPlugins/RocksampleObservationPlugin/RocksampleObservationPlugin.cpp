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
#include "RocksampleObservationPluginOptions.hpp"

namespace oppt
{
class RocksampleObservationPlugin: public ObservationPlugin
{
public :
    RocksampleObservationPlugin():
        ObservationPlugin(),
        rockLocations_() {

    }

    virtual ~RocksampleObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<RocksampleObservationPluginOptions>(optionsFile);        
        halfEfficiencyDistance_ = static_cast<RocksampleObservationPluginOptions*>(options_.get())->halfEfficiencyDistance;
        distr_ = std::make_unique<std::uniform_real_distribution<FloatType>>(0.0, 1.0);
        getRockLocations();
        return true;
    }    

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
        long binNumber = 0;

        if (actionVec[0] > 5.5) {
            // We perform a checkRock action. Get the rock we want to check and its location
            unsigned int rockIndex = (unsigned int)(actionVec[0] + 0.025);
            VectorFloat rockLocation = rockLocations_.at(rockIndex);

            // Get the euclidean distance between the rover and the rockLocation
            FloatType distance = euclideanDistance_(stateVec, rockLocation);

            // Get the probability of receiving a correct sensor reading based on the distance between the rover and the rock
            FloatType probability = getSensorCorrectnessProbability_(distance);

            // Determine whether we recieve a correct observation                   
            bool obsMatches = (*(distr_.get()))((*(robotEnvironment_->getRobot()->getRandomEngine().get()))) <= probability;                
            if (obsMatches) {
                // We will receive a correct observation. If the rock is good (== 1), the observation will be the one with binNumber = 1.
                // If it is bad, the observation will be the one with binNumber = 2.
                (unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1 ? binNumber = 1 : binNumber = 2;
            } else {
                // We will receive an incorrect observation. If the rock is good (== 1), the observation will be the one with binNumber = 2.
                // If it is bad, the observation will be the one with binNumber = 1.
                (unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1 ? binNumber = 2 : binNumber = 1;                
            }
        }

        // Finally, construct the observation as a oppt::DiscreteVectorObservation
        observationResult->observation = ObservationSharedPtr(new DiscreteVectorObservation({(FloatType)(binNumber)}));        
        observationResult->observation->as<DiscreteVectorObservation>()->setBinNumber(binNumber);        
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const override {
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat actionVec = action->as<VectorAction>()->asVector();
        long binNumber = observation->as<DiscreteVectorObservation>()->getBinNumber();
        if (actionVec[0] < 5.5) {
            return 1.0;
        } else {
            // We performed a checkRock action. Get the rock we were checking
            unsigned int rockIndex = (unsigned int)(actionVec[0] + 0.25);
            VectorFloat rockLocation = rockLocations_.at(rockIndex);

            // Get the euclidean distance between the rover and the rockLocation
            FloatType distance = euclideanDistance_(stateVec, rockLocation);

            // Get the probability of receiving a correct sensor reading
            FloatType correctObservationProbability = getSensorCorrectnessProbability_(distance);
            FloatType likelihood = 0.0;

            if (binNumber == 1) {
                // We observed the rock to be good. If the rock was actually good (== 1), the probability of getting the received observation
                // is correctObservationProbability. Otherwise it is 1 - correctObservationProbability.
                (unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1 ? likelihood = correctObservationProbability : likelihood = 1.0 - correctObservationProbability;
            } else {
                // We observed the rock to be bad. If the rock was actually good (== 1), the probability of getting the received observation
                // is 1 - correctObservationProbability. Otherwise it is correctObservationProbability.
                (unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1 ? likelihood = 1.0 - correctObservationProbability : likelihood = correctObservationProbability;
            }           
            
            return likelihood;
        }
    }

private:
    // A map from rock indices to their locations
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

    FloatType halfEfficiencyDistance_;

    std::unique_ptr<std::uniform_real_distribution<FloatType>> distr_ = nullptr;

private:
    void getRockLocations() {
        auto scene = robotEnvironment_->getScene();
        VectorBodyPtr bodies = scene->getBodies();
        unsigned int idx = 6;
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

    FloatType euclideanDistance_(const VectorFloat& stateVec, const VectorFloat& rockLocation) const {
        VectorFloat roverPosition( {stateVec[0], stateVec[1]});
        return math::euclideanDistance<FloatType>(roverPosition, rockLocation);
    }

    FloatType getSensorCorrectnessProbability_(const FloatType& distance) const {
        return (1.0 + std::pow(2.0, -distance / halfEfficiencyDistance_)) * 0.5;
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(RocksampleObservationPlugin)

}
