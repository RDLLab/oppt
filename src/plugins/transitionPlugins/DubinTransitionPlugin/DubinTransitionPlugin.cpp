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
#include "DubinTransitionPluginOptions.hpp"

namespace oppt
{
class DubinTransitionPlugin: public TransitionPlugin
{
public:
    DubinTransitionPlugin():
        TransitionPlugin() {
    }

    virtual ~DubinTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {        
        parseOptions_<DubinTransitionPluginOptions>(optionsFile);
        processError_ =
            static_cast<DubinTransitionPluginOptions*>(options_.get())->processError;
        controlDuration_ =
            static_cast<DubinTransitionPluginOptions*>(options_.get())->controlDuration;
        makeErrorDistribution();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
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
        VectorFloat stateVector = denormalizedInputState->as<VectorState>()->asVector();

        // Add the control error to the action
        VectorFloat affectedActionVec =
            addVectors(propagationRequest->action->as<VectorAction>()->asVector(), errorVector);
        ActionSharedPtr affectedActionNormalized = std::make_shared<VectorAction>(affectedActionVec);
        robot->getActionSpace()->enforceActionLimits(affectedActionNormalized);
        ActionSharedPtr denormalizedAction = robot->getActionSpace()->denormalizeAction(affectedActionNormalized);
        VectorFloat control = denormalizedAction->as<VectorAction>()->asVector();

        // Make the new state vector
        VectorFloat resultingState(4);
        resultingState[0] = stateVector[0] + controlDuration_ * stateVector[3] * cos(stateVector[2]);
        resultingState[1] = stateVector[1] + controlDuration_ * stateVector[3] * sin(stateVector[2]);
        resultingState[2] = stateVector[2] + controlDuration_ * stateVector[3] * tan(control[1]) / d_;
        resultingState[3] = stateVector[3] + controlDuration_ * (control[0]);
        RobotStateSharedPtr dnS = std::make_shared<oppt::VectorState>(resultingState);

        // Check for collisions        
        if (propagationRequest->enableCollision) {
            propagationResult->collisionReport = makeCollisionReport(denormalizedInputState, dnS);;
            if (propagationResult->collisionReport->collides && propagationRequest->allowCollisions) {
                // Make the next state after a collision
                resultingState[0] = stateVector[0];
                resultingState[1] = stateVector[1];
                resultingState[2] = stateVector[2];
                resultingState[3] = -3 * stateVector[3];
            }
        }

        robotEnvironment_->getGazeboInterface()->setStateVector(resultingState);
        auto newWorldState = robotEnvironment_->getGazeboInterface()->getWorldState();

        // Normalize the output state
        RobotStateSharedPtr denormalizedNextState = std::make_shared<oppt::VectorState>(resultingState);
        propagationResult->nextState = robot->getStateSpace()->normalizeState(denormalizedNextState);

        // Enforce the state constraints
        robot->getStateSpace()->enforceStateLimits(propagationResult->nextState);        
        propagationResult->nextState->setGazeboWorldState(newWorldState);
        propagationResult->errorVector = errorVector;
        return propagationResult;
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }

private:
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    FloatType processError_ = 0.0;

    FloatType controlDuration_ = 0.0;

    // Distance between axels
    FloatType d_ = 0.11;

    Matrixdf makeCovarianceMatrix(const FloatType& error) {
        auto actionSpaceDimension = robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions();
        Matrixdf covarianceMatrix = Matrixdf::Identity(actionSpaceDimension, actionSpaceDimension);
        VectorFloat lowerControlLimits;
        VectorFloat upperControlLimits;
        oppt::ActionLimitsSharedPtr actionLimits = robotEnvironment_->getRobot()->getActionSpace()->getActionLimits();
        actionLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerControlLimits, upperControlLimits);
        for (size_t i = 0; i < lowerControlLimits.size(); i++) {
            FloatType controlRange = upperControlLimits[i] - lowerControlLimits[i];
            covarianceMatrix(i, i) = error * error;
        }

        return covarianceMatrix;
    }

    bool makeErrorDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        errorDistribution_ =
            std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        Matrixdf mean = Matrixdf::Zero(robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions(), 1);
        Matrixdf covarianceMatrix = makeCovarianceMatrix(processError_);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        return true;
    }

    CollisionReportSharedPtr makeCollisionReport(const RobotStateSharedPtr& state1,
            const RobotStateSharedPtr& state2) const {
        auto state1N = robotEnvironment_->getRobot()->getStateSpace()->normalizeState(state1);
        auto state2N = robotEnvironment_->getRobot()->getStateSpace()->normalizeState(state2);
        auto robot = robotEnvironment_->getRobot();
        unsigned int n = 5;
        FloatType t = 0;
        CollisionReportSharedPtr collisionReport(new CollisionReport());
        for (unsigned int i = 0; i != n; ++i) {
            t += 1.0 / (FloatType)n;
            auto interpolatedStateN =
                robot->getStateSpace()->interpolate(state1N, state2N, t);
            auto interpolatedStateD =
                robot->getStateSpace()->denormalizeState(interpolatedStateN);
            robotEnvironment_->getGazeboInterface()->setStateVector(interpolatedStateD->as<VectorState>()->asVector());            
            collisionReport = robot->makeDiscreteCollisionReportDirty();
            if (collisionReport->collides)
                return collisionReport;
        }

        return collisionReport;
    }

};

OPPT_REGISTER_TRANSITION_PLUGIN(DubinTransitionPlugin)

}
