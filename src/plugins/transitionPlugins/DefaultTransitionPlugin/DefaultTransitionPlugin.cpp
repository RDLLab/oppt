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
#include "DefaultTransitionPluginOptions.hpp"

namespace oppt
{
class DefaultTransitionPlugin: public TransitionPlugin
{
public :
    DefaultTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~DefaultTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {        
        parseOptions_<DefaultTransitionPluginOptions>(optionsFile);
        controlDuration_ =
            static_cast<DefaultTransitionPluginOptions*>(options_.get())->controlDuration;
        FloatType softLimitThreshold =
            static_cast<DefaultTransitionPluginOptions*>(options_.get())->softLimitThreshold;
        robotEnvironment_->getGazeboInterface()->setSoftLimitThreshold(softLimitThreshold);
        makeErrorDistribution();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        GazeboPropagationRequest *gazeboPropagationRequest = new GazeboPropagationRequest();
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        auto robot = robotEnvironment_->getRobot();
        VectorFloat errorVector;

        // Sample a control error if neccessary
        if (propagationRequest->errorVector.size() != robot->getActionSpace()->getNumDimensions()) {
            errorVector = toStdVec<FloatType>(errorDistribution_->sample(1).col(0));
        } else {
            errorVector = propagationRequest->errorVector;
        }

        // Denormalize the input state
        auto denormalizedInputState =
            robot->getStateSpace()->denormalizeState(propagationRequest->currentState);

        // Add the control error to the action
        VectorFloat affectedActionVec =
            addVectors(propagationRequest->action->as<VectorAction>()->asVector(), errorVector);
        ActionSharedPtr affectedActionNormalized = std::make_shared<VectorAction>(affectedActionVec);
        robot->getActionSpace()->enforceActionLimits(affectedActionNormalized);
        ActionSharedPtr denormalizedAction = robot->getActionSpace()->denormalizeAction(affectedActionNormalized);

        // Build the data structure for the GazeboPropagationReport
        gazeboPropagationRequest->currentStateVec = denormalizedInputState->as<VectorState>()->asVector();
        gazeboPropagationRequest->actionVec = denormalizedAction->as<VectorAction>()->asVector();
        gazeboPropagationRequest->duration = controlDuration_;
        gazeboPropagationRequest->enableCollision = propagationRequest->enableCollision;
        gazeboPropagationRequest->allowCollisions = propagationRequest->allowCollisions;
        gazeboPropagationRequest->currentWorldState = propagationRequest->currentState->getGazeboWorldState();

        // Do the propagation
        GazeboPropagationResultUniquePtr gazeboPropagationResult =
            robotEnvironment_->getGazeboInterface()->doPropagation(gazeboPropagationRequest);

        propagationResult->collisionReport = gazeboPropagationResult->collisionReport;

        // Normalize the output state
        RobotStateSharedPtr denormalizedNextState = std::make_shared<oppt::VectorState>(gazeboPropagationResult->nextStateVec);
        propagationResult->nextState = robot->getStateSpace()->normalizeState(denormalizedNextState);

        propagationResult->nextState->setGazeboWorldState(gazeboPropagationResult->nextWorldState);
        if (gazeboPropagationResult->subStates.size() > 0) {
            std::vector<RobotStateSharedPtr> subStates(gazeboPropagationResult->subStates.size());
            for (size_t i = 0; i != subStates.size(); ++i) {
                subStates[i] = std::make_shared<VectorState>(gazeboPropagationResult->subStates[i]);
            }

            propagationResult->nextState->setSubStates(subStates);
        }

        propagationResult->errorVector = errorVector;

        delete gazeboPropagationRequest;
        return propagationResult;
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }

private:
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    FloatType controlDuration_ = 0;

    Matrixdf makeCovarianceMatrix(const FloatType& error) {
        auto actionSpaceDimension = robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions();
        Matrixdf covarianceMatrix = Matrixdf::Identity(actionSpaceDimension, actionSpaceDimension);
        VectorFloat lowerControlLimits;
        VectorFloat upperControlLimits;
        oppt::ActionLimitsSharedPtr actionLimits = robotEnvironment_->getRobot()->getActionSpace()->getActionLimits();
        actionLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerControlLimits, upperControlLimits);
        for (size_t i = 0; i < lowerControlLimits.size(); i++) {
            covarianceMatrix(i, i) = error * error;
        }

        return covarianceMatrix;
    }

    bool makeErrorDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        auto options = static_cast<DefaultTransitionPluginOptions*>(options_.get());
        if (options->errorDistribution == "GAUSSIAN") {
            errorDistribution_ =
                std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
            Matrixdf mean = Matrixdf::Zero(robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions(), 1);
            Matrixdf covarianceMatrix = makeCovarianceMatrix(options->processError);
            errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
            errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        } else if (options->errorDistribution == "UNIFORM") {
            size_t numActionSpaceDimensions = robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions();
            if (numActionSpaceDimensions != options->lowerUpperBoundUniformDistribution.size()) {
                ERROR("Size of lowerUpperBoundUniformDistribution doesn't match number of action space dimensions (" +
                      std::to_string(numActionSpaceDimensions) + ")");
            }

            VectorFloat lowerBound;
            VectorFloat upperBound;
            for (size_t i = 0; i != numActionSpaceDimensions; ++i) {
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
            ERROR("Type of error distribution in transitionPluginOptions not recognized. Available distributions are GAUSSIAN and UNIFORM");
        }
        return true;
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(DefaultTransitionPlugin)

}
