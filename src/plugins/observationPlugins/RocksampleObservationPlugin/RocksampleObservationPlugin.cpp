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

        // The observationError parameter acts as the efficiency distance here
        observationError_ =
            static_cast<RocksampleObservationPluginOptions*>(options_.get())->observationError;
        getRockLocations();
        return true;
    }

    void sampleRockAtRobotPosition() {

    }

    FloatType manhattanDistance(const VectorFloat& p1, const VectorFloat& p2) const {
        return std::fabs(p2[0] - p1[0]) + std::fabs(p2[1] - p1[1]);
    }

    FloatType euclideanDist(const VectorFloat& stateVec, const VectorFloat& rockLocation) const {
        VectorFloat robotPosition( {stateVec[0], stateVec[1]});
        return math::euclideanDistance<FloatType>(robotPosition, rockLocation);
    }

    FloatType getSensorCorrectnessProbability(const FloatType& distance) const {
        return (1.0 + std::pow(2.0, -distance / observationError_)) * 0.5;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();

        // Get the observation
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
        VectorFloat observationVec(robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions(), 0.0);
        long binNumber = 0;

        if (actionVec[0] < 5.5) {
            // The action is a movement action, so no sampling here. An observation of -1 indicates a null observation
            observationVec[0] = -1.0;
        } else {
            // Get the rock we want to check
            unsigned int rockIndex = (unsigned int)(actionVec[0] + 0.025);
            VectorFloat rockLocation = rockLocations_.at(rockIndex);

            // Get euclidean distance to rockLocation
            FloatType distance = euclideanDist(stateVec, rockLocation);
            FloatType probability = getSensorCorrectnessProbability(distance);
            bool obsMatches =
                std::bernoulli_distribution(
                    probability)(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
            if (obsMatches) {
                observationVec[0] = stateVec[rockIndex - 4];
                if ((unsigned int)(observationVec[0] + 0.25) == 1)
                    binNumber = 2;
                else {
                    binNumber = 1;
                }
            } else {
                if ((unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1) {
                    observationVec[0] = 0.0;
                    binNumber = 1;
                } else {
                    observationVec[0] = 1.0;
                    binNumber = 2;
                }
            }
        }

        auto observationSpace = robotEnvironment_->getRobot()->getObservationSpace();
        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(observationVec);
        observation->as<DiscreteVectorObservation>()->setBinNumber(binNumber);
        observationResult->observation = observation;
        observationResult->errorVector = observationRequest->errorVector;
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const override {
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat actionVec = action->as<VectorAction>()->asVector();
        VectorFloat observationVec = observation->as<DiscreteVectorObservation>()->asVector();
        if (actionVec[0] < 5.5) {
            return 1.0;
        } else {
            // Action is a check action

            // Get the rock we want to check
            unsigned int rockIndex = (unsigned int)(actionVec[0] + 0.25);
            VectorFloat rockLocation = rockLocations_.at(rockIndex);
            FloatType distance = euclideanDist(stateVec, rockLocation);
            FloatType probability = getSensorCorrectnessProbability(distance);
            FloatType likelihood = 0.0;
            if ((unsigned int)(observationVec[0] + 0.25) == 0) {
                if ((unsigned int)(stateVec[rockIndex - 4] + 0.25) == 0) {
                    likelihood = probability;
                } else {
                    likelihood = 1.0 - probability;
                }
            } else {
                if ((unsigned int)(stateVec[rockIndex - 4] + 0.25) == 1) {
                    likelihood = probability;
                } else {
                    likelihood = 1.0 - probability;
                }
            }

            if (likelihood < 0.0001)
                return 0.000001;
            return likelihood;
        }
    }

private:
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

    FloatType observationError_;

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
};

OPPT_REGISTER_OBSERVATION_PLUGIN(RocksampleObservationPlugin)

}
