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
#include "oppt/robotHeaders/ActionSpaceDiscretizer.hpp"
#include "oppt/robotHeaders/ActionSpace.hpp"

namespace oppt
{
ActionSpaceDiscretizer::ActionSpaceDiscretizer(ActionSpaceSharedPtr& actionSpace):
    actionSpace_(actionSpace)
{

}

std::vector<ActionSharedPtr> ActionSpaceDiscretizer::getAllActionsInOrder(const unsigned int& numStepsPerDimension) const
{
    ActionLimitsSharedPtr actionLimits = actionSpace_->getActionLimits();
    VectorFloat lowerLimits;
    VectorFloat upperLimits;
    unsigned int numDimensions = actionSpace_->getNumDimensions();
    if (!actionLimits) {
        ERROR("action limits is null");
    }

    std::vector<ActionSharedPtr> allActionsOrdered_ = std::vector<ActionSharedPtr>(std::pow(numStepsPerDimension, numDimensions));
    actionLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerLimits, upperLimits);
    for (long code = 0; code < std::pow(numStepsPerDimension, numDimensions); code++) {
        VectorFloat ks;
        VectorFloat ss;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            ks.push_back((upperLimits[i] - lowerLimits[i]) / (numStepsPerDimension - 1));
        }

        FloatType j = code;
        FloatType j_old = code;
        FloatType s = 0;
        for (size_t i = lowerLimits.size() - 1; i != (size_t) - 0; i--) {
            FloatType s;
            j = j_old / std::pow(numStepsPerDimension, i);
            modf(j, &s);
            ss.push_back(s);
            if (i != 1) {
                j = (int)(j_old) % (int)std::pow(numStepsPerDimension, i);
                j_old = j;
            }
        }

        ss.push_back((int)j_old % numStepsPerDimension);
        VectorFloat actionValues;
        for (size_t i = 0; i < lowerLimits.size(); i++) {
            actionValues.push_back(lowerLimits[i] + ss[i] * ks[i]);
        }

        ActionSharedPtr action(new DiscreteVectorAction(actionValues));
        //ActionSharedPtr normalizedAction;
        //this->normalizeAction(action, normalizedAction);
        action->as<DiscreteVectorAction>()->setBinNumber(code);
        allActionsOrdered_[code] = action;
    }

    return allActionsOrdered_;
}

CustomActionSpaceDiscretizer::CustomActionSpaceDiscretizer(ActionSpaceSharedPtr& actionSpace,
        const std::vector<unsigned int>& discretization):
    ActionSpaceDiscretizer(actionSpace),
    discretization_(discretization)
{

}

std::vector<ActionSharedPtr> CustomActionSpaceDiscretizer::getAllActionsInOrder(const unsigned int& numStepsPerDimension) const
{
    if (actionSpace_->getNumDimensions() != discretization_.size())
        ERROR("Discretization vector has different size that the action space");
    ActionLimitsSharedPtr actionLimits = actionSpace_->getActionLimits();
    VectorFloat lowerLimits;
    VectorFloat upperLimits;
    actionLimits->getLimits()->as<VectorLimitsContainer>()->get(lowerLimits, upperLimits);
    unsigned int numActions = discretization_[0];
    for (size_t i = 1; i != discretization_.size(); ++i) {
        numActions *= discretization_[i];
    }
    std::vector<ActionSharedPtr> allActionsOrdered_ = std::vector<ActionSharedPtr>(numActions);

    VectorFloat actionVals(lowerLimits);

    ActionSharedPtr action(new DiscreteVectorAction(actionVals));
    long code = 0;
    action->as<DiscreteVectorAction>()->setBinNumber(code);
    allActionsOrdered_[code] = action;

    int currIndex = 0;
    while (code != numActions - 1) {
        actionVals[currIndex] += ((upperLimits[currIndex] - lowerLimits[currIndex]) / ((FloatType)(discretization_[currIndex]) - 1));
        if (actionVals[currIndex] > upperLimits[currIndex]) {
            while (true) {
                currIndex++;
                actionVals[currIndex] += ((upperLimits[currIndex] - lowerLimits[currIndex]) / ((FloatType)(discretization_[currIndex]) - 1));
                if (actionVals[currIndex] <= upperLimits[currIndex])
                    break;
            }

            for (size_t i = 0; i != currIndex; ++i) {
                actionVals[i] = lowerLimits[i];
            }
            
            currIndex = 0;
        }

        code++;
        ActionSharedPtr action(new DiscreteVectorAction(actionVals));	
        action->as<DiscreteVectorAction>()->setBinNumber(code);
        allActionsOrdered_[code] = action;	
    }
    
    return allActionsOrdered_;
}

}
