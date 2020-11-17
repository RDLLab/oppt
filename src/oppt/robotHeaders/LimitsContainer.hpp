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
#ifndef __LIMITS_CONTAINER_HPP__
#define __LIMITS_CONTAINER_HPP__

namespace oppt
{
class LimitsContainer
{
public:
    LimitsContainer() = default;

    virtual ~LimitsContainer() = default;

    _NO_COPY_BUT_MOVE(LimitsContainer)

    _STATIC_CAST

};

class VectorLimitsContainer: public LimitsContainer
{
public:
    VectorLimitsContainer(VectorFloat& lowerLimits, VectorFloat& upperLimits):
        LimitsContainer(),
        lowerLimits_(lowerLimits),
        upperLimits_(upperLimits) {

    }

    virtual void get(VectorFloat& lowerLimits, VectorFloat& upperLimits) const {
        lowerLimits = lowerLimits_;
        upperLimits = upperLimits_;
    }


private:
    VectorFloat lowerLimits_;
    VectorFloat upperLimits_;

};

}

#endif
