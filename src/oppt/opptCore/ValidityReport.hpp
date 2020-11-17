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
#ifndef __OPPT_VALIDITY_REPORT_HPP__
#define __OPPT_VALIDITY_REPORT_HPP__
#include "typedefs.hpp"

namespace oppt
{
/**
 * Datastructure that represents a state validity report.
 */
struct ValidityReport {
    _NO_COPY_BUT_MOVE(ValidityReport)

    /** @brief Construct from a oppt::RobotStateSharedPtr */
    ValidityReport(const RobotStateSharedPtr& state):
        state_(state) {

    }

    virtual ~ValidityReport() = default;
    
    /** @brief The oppt::RobotStateSharedPtr this validity report corresponds to */    
    const RobotStateSharedPtr state_;

    /** @brief Is true iff state_ is a valid state */
    bool isValid = false;

    /** @brief Is true iff state_ collides */
    bool collided = false;

    /** @brief Is true iff state_ is within the state space constraints */
    bool satisfiesConstraints = false;

};

/** @brief std::shared_ptr to oppt::ValidityReport*/
typedef std::shared_ptr<ValidityReport> ValidityReportSharedPtr;
}

#endif
