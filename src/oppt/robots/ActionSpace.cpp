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
#include <memory>

using std::cout;
using std::endl;

namespace oppt
{
ActionSpace::ActionSpace(const ActionSpaceInfo& actionSpaceInfo):
    numDimensions_(actionSpaceInfo.numDimensions),
    actionLimits_(nullptr),
    denormalizedActionLimits_(nullptr),
    actionSpaceInfo_(actionSpaceInfo),
    actionSpaceInformation_(nullptr)
{

}

const ActionSpaceInfo ActionSpace::getInfo() const
{
    return actionSpaceInfo_;
}

size_t ActionSpace::getNumDimensions() const
{
    return numDimensions_;
}

bool ActionSpace::enforceActionLimits(ActionSharedPtr& action) const {
    actionLimits_->enforceLimits(action);
    return true;
}

ActionLimitsSharedPtr ActionSpace::getActionLimits() const
{
    return actionLimits_;
}

ActionSharedPtr ActionSpace::normalizeAction(const ActionSharedPtr& action) const {
    if (!actionNormalizer_)
        return action;
    return actionNormalizer_->normalizeAction(action);
}

ActionSharedPtr ActionSpace::denormalizeAction(const ActionSharedPtr& action) const {
    if (!actionNormalizer_)
        return action;
    return actionNormalizer_->denormalizeAction(action);
}

void ActionSpace::setActionNormalizer(std::unique_ptr<ActionNormalizer> actionNormalizer) {
    actionNormalizer_ = std::move(actionNormalizer);
}

}
