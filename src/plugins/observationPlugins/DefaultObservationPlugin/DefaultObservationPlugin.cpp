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
#include "DefaultObservationPluginOptions.hpp"

namespace oppt
{
class DefaultObservationPlugin: public ObservationPlugin
{
public :
    DefaultObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~DefaultObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {        
        parseOptions_<DefaultObservationPluginOptions>(optionsFile);
        makeErrorDistribution();
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        GazeboObservationRequest* gazeboObservationRequest = new GazeboObservationRequest();
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();
        VectorFloat errorVector;
        auto robot = robotEnvironment_->getRobot();

        // Sample an observation error is neccessary
        if (observationRequest->errorVector.size() != robot->getObservationSpace()->getNumDimensions()) {
            errorVector = toStdVec<FloatType>(errorDistribution_->sample(1));
        } else {
            errorVector = observationRequest->errorVector;
        }

        //printVector(errorVector, "errorVector");

        // Denormalize the state
        RobotStateSharedPtr denormalizedState =
            robot->getStateSpace()->denormalizeState(observationRequest->currentState);

        gazeboObservationRequest->currentStateVec = denormalizedState->as<VectorState>()->asVector();
        gazeboObservationRequest->currentWorldState = observationRequest->currentState->getGazeboWorldState();
        GazeboObservationResultUniquePtr gazeboObservationResult =
            robotEnvironment_->getGazeboInterface()->makeObservationReport(gazeboObservationRequest);

        ObservationSharedPtr denormalizedObservation = std::make_shared<VectorObservation>(gazeboObservationResult->observationVec);
        ObservationSharedPtr normalizedObservation =
            robot->getObservationSpace()->normalizeObservation(denormalizedObservation);
        VectorFloat normalizedObservationVec = normalizedObservation->as<VectorObservation>()->asVector();

        // Add the observation error to the observationVector
        ObservationSharedPtr affectedObservation = std::make_shared<VectorObservation>(addVectors(normalizedObservationVec,
                errorVector));

        // Normalize the observation
        observationResult->observation = affectedObservation;
        observationResult->errorVector = errorVector;
        delete gazeboObservationRequest;
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr &state,
                                     const Action *action,
                                     const Observation *observation) const override {
        // Get the nominal observation
        ObservationRequestSharedPtr observationRequest(new ObservationRequest());
        VectorFloat zZero(robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions(), 0);
        observationRequest->currentState = state;
        observationRequest->action = action;
        observationRequest->errorVector = zZero;
        ObservationResultSharedPtr observationResult = robotEnvironment_->getRobot()->makeObservationReport(observationRequest);
        VectorFloat observationVec = observationResult->observation->as<VectorObservation>()->asVector();
        //VectorFloat actObservationVec = observation->as<VectorObservation>()->asVector()
        VectorFloat diff =
            subtractVectors<FloatType>(observation->as<VectorObservation>()->asVector(), observationVec);
        return errorDistribution_->pdf(diff);
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }

private:
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

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
        auto options = static_cast<DefaultObservationPluginOptions*>(options_.get());
        if (options->errorDistribution == "GAUSSIAN") {
            errorDistribution_ =
                std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
            Matrixdf mean = Matrixdf::Zero(robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions(), 1);
            Matrixdf covarianceMatrix = makeCovarianceMatrix(options->observationError);
            errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
            errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        } else if (options->errorDistribution == "UNIFORM") {
            size_t numObservationSpaceDimensions = robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
            if (numObservationSpaceDimensions != options->lowerUpperBoundUniformDistribution.size()) {
                ERROR("Size of lowerUpperBoundUniformDistribution doesn't match number of observation space dimensions (" +
                      std::to_string(numObservationSpaceDimensions) + ")");
            }

            VectorFloat lowerBound;
            VectorFloat upperBound;
            for (size_t i = 0; i != numObservationSpaceDimensions; ++i) {
                if (options->lowerUpperBoundUniformDistribution[i].size() != 2)
                    ERROR("Size of element '" + std::to_string(i) + "' of lowerUpperBoundUniformDistribution doesn't equal 2");
                if (options->lowerUpperBoundUniformDistribution[i][0] > options->lowerUpperBoundUniformDistribution[i][1])
                    ERROR("Lower bound in lowerUpperBoundUniformDistribution must be smaller than upper bound");
                lowerBound.push_back(options->lowerUpperBoundUniformDistribution[i][0]);
                upperBound.push_back(options->lowerUpperBoundUniformDistribution[i][1]);
            }            
            
            errorDistribution_ =
                std::unique_ptr<UniformDistribution<FloatType>>(new UniformDistribution<FloatType>(lowerBound, upperBound, randomEngine));
        } else {
            ERROR("Type of error distribution in observationPluginOptions not recognized. Available distributions are GAUSSIAN and UNIFORM");
        }

        return true;
    }
};

OPPT_REGISTER_OBSERVATION_PLUGIN(DefaultObservationPlugin)

}
