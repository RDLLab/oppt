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
#include "oppt/robotHeaders/ActionSpace.hpp"

namespace oppt
{

ActionSharedPtr VectorActionNormalizer::normalizeAction(const ActionSharedPtr& action) const {
    VectorFloat lowerActionLimits;
    VectorFloat upperActionLimits;
    actionLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerActionLimits, upperActionLimits);
    VectorFloat actionVec = static_cast<VectorAction*>(action.get())->asVector();
    for (size_t i = 0; i < actionVec.size(); i++) {
        actionVec[i] = (actionVec[i] - lowerActionLimits[i]) / (upperActionLimits[i] - lowerActionLimits[i]);
    }

    return std::make_shared<VectorAction>(actionVec);
}

ActionSharedPtr VectorActionNormalizer::denormalizeAction(const ActionSharedPtr& action) const {
    VectorFloat lowerActionLimits;
    VectorFloat upperActionLimits;
    actionLimits_->getRawLimits()->as<VectorLimitsContainer>()->get(lowerActionLimits, upperActionLimits);
    VectorFloat actionVec = static_cast<VectorAction*>(action.get())->asVector();
    for (size_t i = 0; i < actionVec.size(); i++) {
        actionVec[i] = actionVec[i] * (upperActionLimits[i] - lowerActionLimits[i]) + lowerActionLimits[i];
    }

    return std::make_shared<VectorAction>(actionVec);
}

ContinuousVectorActionSpace::ContinuousVectorActionSpace(const ActionSpaceInfo& actionSpaceInfo):
    ContinuousActionSpace(actionSpaceInfo)
{
    if (actionSpaceInfo.normalized) 
        actionNormalizer_ = std::unique_ptr<ActionNormalizer>(new VectorActionNormalizer());
}

void ContinuousVectorActionSpace::makeActionLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits)
{
    LimitsContainerSharedPtr container = std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
    LimitsContainerSharedPtr containerDenormalized = std::make_shared<VectorLimitsContainer>(lowerLimits, upperLimits);
    denormalizedActionLimits_ = std::make_shared<VectorActionLimits>(containerDenormalized);
    if (actionSpaceInfo_.normalized) {
        actionLimits_ = std::make_shared<NormalizedVectorActionLimits>(container);
    } else {
        actionLimits_ = std::make_shared<VectorActionLimits>(container);
    }

    if (actionNormalizer_)
        actionNormalizer_->setActionLimits(actionLimits_);
}

VectorFloat ContinuousVectorActionSpace::getOrigin() const
{
    if (actionSpaceInfo_.normalized) {
        VectorFloat origin(actionSpaceInfo_.numDimensions);
        VectorFloat lowerActionLimits;
        VectorFloat upperActionLimits;
        denormalizedActionLimits_->getLimits()->as<VectorLimitsContainer>()->get(lowerActionLimits, upperActionLimits);
        for (size_t i = 0; i < actionSpaceInfo_.numDimensions; i++) {
            origin[i] = -(lowerActionLimits[i] / (upperActionLimits[i] - lowerActionLimits[i]));
        }

        return origin;
    }

    return VectorFloat(actionSpaceInfo_.numDimensions, 0);
}

ActionSharedPtr ContinuousVectorActionSpace::sampleUniform(std::default_random_engine* randGen) const
{
    VectorFloat lowerActionLimits;
    VectorFloat upperActionLimits;
    actionLimits_->getLimits()->as<VectorLimitsContainer>()->get(lowerActionLimits, upperActionLimits);
    VectorFloat randomActionVec(lowerActionLimits.size());
    for (size_t i = 0; i < lowerActionLimits.size(); i++) {
        std::uniform_real_distribution<FloatType> uniform_dist(lowerActionLimits[i], upperActionLimits[i]);
        FloatType rand_num = uniform_dist(*randGen);
        randomActionVec[i] = rand_num;
    }

    ActionSharedPtr action = std::make_shared<VectorAction>(randomActionVec);
    return action;
}

}
