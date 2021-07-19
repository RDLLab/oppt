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
#include "DubinObservationPluginOptions.hpp"

namespace oppt
{
class DubinObservationPlugin: public ObservationPlugin
{
public :
    DubinObservationPlugin():
        ObservationPlugin() {
        b1_ = VectorFloat( { -0.7, 0.7});
        b2_ = VectorFloat( {0.7, -0.7});
    }

    virtual ~DubinObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {        
        parseOptions_<DubinObservationPluginOptions>(optionsFile);
        observationError_ =
            static_cast<DubinObservationPluginOptions*>(options_.get())->observationError;
        makeErrorDistribution();
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();
        VectorFloat errorVector;
        auto robot = robotEnvironment_->getRobot();

        // Sample an observation error is neccessary
        if (observationRequest->errorVector.size() != robot->getObservationSpace()->getNumDimensions()) {
            errorVector = toStdVec<FloatType>(errorDistribution_->sample(1));
        } else {
            errorVector = observationRequest->errorVector;
        }

        // Denormalize the state
        RobotStateSharedPtr denormalizedState =
            robot->getStateSpace()->denormalizeState(observationRequest->currentState);

        // Get the observation
        VectorFloat stateVec = denormalizedState->as<VectorState>()->asVector();
        VectorFloat observationVec(3);
        observationVec[0] = 1.0 / (std::pow(stateVec[0] - b1_[0], 2) + std::pow(stateVec[1] - b1_[1], 2) + 1.0);
        observationVec[1] = 1.0 / (std::pow(stateVec[0] - b2_[0], 2) + std::pow(stateVec[1] - b2_[1], 2) + 1.0);
        observationVec[2] = stateVec[3];

        // Normalize observation
        ObservationSharedPtr denormalizedObservation =
            std::make_shared<VectorObservation>(observationVec);
        ObservationSharedPtr normalizedObservation =
            robot->getObservationSpace()->normalizeObservation(denormalizedObservation);
        VectorFloat normalizedObservationVec = normalizedObservation->as<VectorObservation>()->asVector();

        // Add the observation error to the observationVector
        ObservationSharedPtr affectedObservation = std::make_shared<VectorObservation>(addVectors(normalizedObservationVec,
                errorVector));
        observationResult->observation = affectedObservation;
        observationResult->errorVector = observationRequest->errorVector;
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const override {
        // Get the nominal observation
        ObservationRequestSharedPtr observationRequest(new ObservationRequest());
        VectorFloat zZero(robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions(), 0);
        observationRequest->currentState = state;
        observationRequest->action = action;
        observationRequest->errorVector = zZero;
        ObservationResultSharedPtr observationResult = getObservation(observationRequest.get());
        VectorFloat observationVec = observationResult->observation->as<VectorObservation>()->asVector();
        VectorFloat diff =
            subtractVectors<FloatType>(observation->as<VectorObservation>()->asVector(), observationVec);
        return errorDistribution_->pdf(diff);
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }

private:
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    FloatType observationError_;

    VectorFloat b1_;
    VectorFloat b2_;

private:

    Matrixdf makeCovarianceMatrix(const FloatType& error) {
        unsigned int observationSpaceDimension =
            robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
        Matrixdf covarianceMatrix = Matrixdf::Identity(observationSpaceDimension, observationSpaceDimension);
        VectorFloat lowerObservationLimits;
        VectorFloat upperObservationLimits;
        ObservationLimitsSharedPtr observationLimits =
            robotEnvironment_->getRobot()->getObservationSpace()->getObservationLimits();
        observationLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerObservationLimits, upperObservationLimits);
        for (size_t i = 0; i < lowerObservationLimits.size(); i++) {
            FloatType observation_range = upperObservationLimits[i] - lowerObservationLimits[i];
            covarianceMatrix(i, i) = error * error;
        }

        return covarianceMatrix;
    }

    bool makeErrorDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        errorDistribution_ =
            std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        Matrixdf mean = Matrixdf::Zero(robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions(), 1);
        Matrixdf covarianceMatrix = makeCovarianceMatrix(observationError_);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        return true;
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(DubinObservationPlugin)

}
