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
#ifndef __OPPT_GOAL_HPP__
#define __OPPT_GOAL_HPP__
#include "typedefs.hpp"
#include "utils.hpp"

namespace oppt
{

/**
 * Abstract class that represents a goal the robot has to reach.
 * The only requirement for a concrete implementation is the oppt::Goal::isSatisfied method
 */
class Goal
{
public:
    _NO_COPY_BUT_MOVE(Goal)
    Goal() = default;
    virtual ~Goal() = default;

    _STATIC_CAST
    
    /**
     * Determines if a state satisfies the goal requirements, i.e. if it is a goal state
     * @param state oppt::RobotStateSharedPtr for which the goal requirement is being checked
     * @return True iff state is a goal state
     */
    virtual bool isSatisfied(oppt::RobotStateSharedPtr& state) const = 0;

};

/**
 * Represents a goal that contains spatial information (i.e. the goal is located somewhere in the environment)
 */
class SpatialGoal: public Goal
{
public:
    /**
     * Construct from a ND-vector that contains the center of the goal
     */
    SpatialGoal(VectorFloat& center):
        Goal(),
        center_(center) {

    }
    
    /**
     * Get the distance of a point to the center of the goal
     * @param point The ND-point for which the distance to the center is calculated
     */
    virtual FloatType distanceCenter(const VectorFloat& point) const = 0;

    virtual void getGoalArea(VectorFloat& goalArea) const = 0;

protected:
    VectorFloat center_;

};

/**
 * Represents a goal that is a n-dimensional sphere inside some space
 */
class SphereGoal: public SpatialGoal
{
public:
    /** @brief Construct from the center and the radius of the sphere */
    SphereGoal(VectorFloat& center, FloatType& radius):
        SpatialGoal(center),
        radius_(radius) {

    }

    virtual FloatType distanceCenter(const VectorFloat& point) const override {
        return math::euclideanDistance(point, center_);
    }

    virtual bool isSatisfied(oppt::RobotStateSharedPtr& state) const override {
        return false;
    }

    virtual FloatType isSatisfied(const VectorFloat& point) const {
        if (distanceCenter(point) < radius_) {
            return true;
        }

        return false;
    }

    virtual void getGoalArea(VectorFloat& goalArea) const override {
        goalArea = VectorFloat(4);
        goalArea[0] = center_[0];
        goalArea[1] = center_[1];
        goalArea[2] = center_[2];
        goalArea[3] = radius_;
    }

protected:
    FloatType radius_;

};

}

#endif
