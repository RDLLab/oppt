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
#ifndef _ACTION_SPACE_DISCRETIZER_HPP_
#define _ACTION_SPACE_DISCRETIZER_HPP_
#include "Action.hpp"

namespace oppt
{
class ActionSpaceDiscretizer
{
public:
    ActionSpaceDiscretizer(ActionSpaceSharedPtr &actionSpace);
    
    virtual ~ActionSpaceDiscretizer() = default;
    
    virtual std::vector<ActionSharedPtr> getAllActionsInOrder(const unsigned int& numStepsPerDimension) const;
    
protected:
    ActionSpaceSharedPtr actionSpace_ = nullptr;

};

class CustomActionSpaceDiscretizer: public ActionSpaceDiscretizer {
public:
    CustomActionSpaceDiscretizer(ActionSpaceSharedPtr& actionSpace, const std::vector<unsigned int> &discretization);
    
    virtual ~CustomActionSpaceDiscretizer() = default;
    
    virtual std::vector<ActionSharedPtr> getAllActionsInOrder(const unsigned int& numStepsPerDimension) const override;
    
protected:
    std::vector<unsigned int> discretization_;
    
};

}

#endif
