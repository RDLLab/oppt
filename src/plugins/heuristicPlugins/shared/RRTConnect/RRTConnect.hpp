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
#ifndef _RRT_CONNECT_HPP_
#define _RRT_CONNECT_HPP_
#include "Tree.hpp"

namespace oppt
{
enum ExtendStatus {
    TRAPPED,
    ADVANCED,
    REACHED

};

typedef std::function<RobotStateSharedPtr(const RobotStateSharedPtr &, const RobotStateSharedPtr&, const FloatType&)> InterpolationFunction;
typedef std::function<bool(const RobotStateSharedPtr&)> IsValidFunction;

class RRTConnect
{
public:
    RRTConnect(const RobotEnvironment* robotEnvironment, const FloatType& planningRange);

    ~RRTConnect();

    TrajectorySharedPtr solve(const RobotStateSharedPtr& state, 
                              const FloatType& timeout);

    void setGoalStates(const std::vector<VectorFloat>& goalStates);
    
    void setIsValidFunction(IsValidFunction isValidFn);

    void setDistanceFunction(DistanceFunction distanceFunction);

    void setInterpolationFunction(InterpolationFunction interpolationFunction);

private:
    std::unique_ptr<RRTTree> startTree_;

    std::unique_ptr<RRTTree> goalTree_;

    std::vector<VectorFloat> goalStates_;

    const RobotEnvironment* robotEnvironment_ = nullptr;

    const FloatType planningRange_;

    FloatType goalBias_;

    DistanceFunction distanceFunction_ = nullptr;

    InterpolationFunction interpolationFunction_ = nullptr;

    IsValidFunction isValidFn_ = nullptr;    

private:
    bool satisfiesGoal_(const Node* node) const;

    const RobotStateSharedPtr sample_() const;

    const RobotStateSharedPtr sampleUniform_() const;

    std::pair<ExtendStatus, const Node*> extend_(RRTTree* tree,
            const RobotStateSharedPtr& qRand,
            const Node* nearestNeighbour = nullptr);

    std::pair<ExtendStatus, const Node*> connect_(RRTTree* tree,
            const RobotStateSharedPtr& qRand);

    TrajectorySharedPtr generateTrajectory_(const Node* nodeTree1, const Node* nodeTree2);

    void shortenPath_(TrajectorySharedPtr& trajectory);    

    const RobotStateSharedPtr interpolate_(const RobotStateSharedPtr& state1,
                                           const RobotStateSharedPtr& state2,
                                           const FloatType& t) const;    
};
}

#endif
