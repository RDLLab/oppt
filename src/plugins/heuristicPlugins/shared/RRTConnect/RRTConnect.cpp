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
#include "RRTConnect.hpp"
#include "Tree.hpp"
#include "oppt/opptCore/trajectory.hpp"
#include "oppt/robotEnvironment/include/RobotEnvironment.hpp"
#include "oppt/opptCore/utils.hpp"
#include <algorithm>

namespace oppt
{
RRTConnect::RRTConnect(const RobotEnvironment* robotEnvironment, const FloatType& planningRange):
    startTree_(new RRTTree()),
    goalTree_(new RRTTree()),
    goalStates_(),
    robotEnvironment_(robotEnvironment),
    planningRange_(planningRange),
    goalBias_(0.075)
{
    // Set the default distance function
    distanceFunction_ = DistanceFunction([this](const RobotStateSharedPtr & s1, const RobotStateSharedPtr & s2) {
        return robotEnvironment_->getRobot()->getStateSpace()->distance(s1, s2);
    });

    startTree_->setDistanceFunction(distanceFunction_);
    goalTree_->setDistanceFunction(distanceFunction_);

    // Set default interpolation function
    interpolationFunction_ = InterpolationFunction([this](const RobotStateSharedPtr & s1, const RobotStateSharedPtr & s2, const FloatType & t) {
        return robotEnvironment_->getRobot()->getStateSpace()->interpolate(s1, s2, t);
    });

    // Set default is valid function
    isValidFn_ = IsValidFunction([this](const RobotStateSharedPtr & state) {
        PropagationResultSharedPtr propagationResult(new PropagationResult);
        propagationResult->nextState = state;
        return robotEnvironment_->isValid(propagationResult);
    });
}

RRTConnect::~RRTConnect()
{

}

void RRTConnect::setGoalStates(const std::vector< VectorFloat >& goalStates)
{
    goalStates_ = goalStates;
}

bool RRTConnect::satisfiesGoal_(const Node* node) const
{
    PropagationResultSharedPtr propRes(new PropagationResult());
    propRes->nextState = node->getRobotState();
    return robotEnvironment_->isTerminal(propRes);
}

const RobotStateSharedPtr RRTConnect::sample_() const
{
    return sampleUniform_();
}


const RobotStateSharedPtr RRTConnect::sampleUniform_() const
{
    auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
    return robotEnvironment_->getRobot()->getStateSpace()->sampleUniform(randomEngine);
}

std::pair<ExtendStatus, const Node*> RRTConnect::extend_(RRTTree* tree,
        const RobotStateSharedPtr& qRand,
        const Node* nearestNeighbour)
{
    if (!nearestNeighbour) {
        nearestNeighbour = tree->nearestNeighbour(qRand);
    }

    FloatType d = distanceFunction_(nearestNeighbour->getRobotState(),
                                    qRand);
    if (d <= planningRange_) {
        if (!isValidFn_(qRand))
            return std::pair<ExtendStatus, const Node *>(TRAPPED, nearestNeighbour);
        return std::pair<ExtendStatus, const Node*>(REACHED,
                tree->allocNode(nearestNeighbour, qRand));
    }

    const RobotStateSharedPtr nnVec = interpolationFunction_(nearestNeighbour->getRobotState(),
                                      qRand,
                                      planningRange_ / d);
    nnVec->setGazeboWorldState(nearestNeighbour->getRobotState()->getGazeboWorldState());
    if (isValidFn_(nnVec)) {
        return std::pair<ExtendStatus, const Node*>(ADVANCED,
                tree->allocNode(nearestNeighbour,
                                nnVec));
    }

    return std::pair<ExtendStatus, const Node*>(TRAPPED,
            nearestNeighbour);
}

std::pair<ExtendStatus, const Node*> RRTConnect::connect_(RRTTree* tree,
        const RobotStateSharedPtr& qRand)
{    
    auto nearestNeighbour = tree->nearestNeighbour(qRand);    
    while (true) {
        auto extendResult = extend_(tree, qRand, nearestNeighbour);
        if (extendResult.first == REACHED || extendResult.first == TRAPPED) {
            return extendResult;
        }

        nearestNeighbour = extendResult.second;
    }
}

void RRTConnect::setIsValidFunction(IsValidFunction isValidFn) {
    isValidFn_ = isValidFn;
}

void RRTConnect::setDistanceFunction(DistanceFunction distanceFunction) {
    distanceFunction_ = distanceFunction;
    startTree_->setDistanceFunction(distanceFunction);
    goalTree_->setDistanceFunction(distanceFunction);
}

void RRTConnect::setInterpolationFunction(InterpolationFunction interpolationFunction) {
    interpolationFunction_ = interpolationFunction;
}

TrajectorySharedPtr RRTConnect::solve(const RobotStateSharedPtr& state, const FloatType& timeout)
{
    startTree_->reset();
    goalTree_->reset();
    startTree_->makeRoot(state);
    RobotStateSharedPtr goalState(new VectorState(goalStates_[0]));
    goalState->setGazeboWorldState(state->getGazeboWorldState());
    goalTree_->makeRoot(goalState);

    bool solved = false;
    bool startTree = true;
    FloatType t0 = oppt::clock_ms();
    while (!solved) {
        RRTTree* currentTree = startTree ? startTree_.get() : goalTree_.get();
        RRTTree* otherTree = startTree ? goalTree_.get() : startTree_.get();

        bool sampledValid = false;
        RobotStateSharedPtr qRand = nullptr;
        while (!sampledValid) {
            qRand = sample_();
            qRand->setGazeboWorldState(state->getGazeboWorldState());
            sampledValid = true;
        }

        auto extendResult = extend_(currentTree, qRand);
        if (extendResult.first != TRAPPED) {
            // Try to connect to the other tree
            auto connectResult = connect_(otherTree,
                                          extendResult.second->getRobotState());
            if (connectResult.first == REACHED) {
                if (startTree) {
                    return generateTrajectory_(extendResult.second,
                                               connectResult.second);
                }

                return generateTrajectory_(connectResult.second,
                                           extendResult.second);
            }
        }

        startTree = !startTree;        
        if ((oppt::clock_ms() - t0) / 1000.0 > timeout) {
            return nullptr;
        }
    }

    // Should never get here
    return nullptr;
}

TrajectorySharedPtr RRTConnect::generateTrajectory_(const Node* nodeTree1,
        const Node* nodeTree2)
{
    TrajectorySharedPtr trajectory(new Trajectory());
    auto currNode = nodeTree1;
    while (currNode) {
        trajectory->stateTrajectory.push_back(currNode->getRobotState());
        currNode = currNode->getParent();
    }
    std::reverse(trajectory->stateTrajectory.begin(), trajectory->stateTrajectory.end());

    currNode = nodeTree2->getParent();
    while (currNode) {
        trajectory->stateTrajectory.push_back(currNode->getRobotState());
        currNode = currNode->getParent();
    }

    //shortenPath_(trajectory);
    return trajectory;
}

void RRTConnect::shortenPath_(TrajectorySharedPtr& trajectory)
{
    std::vector<unsigned int> markForDelete;
    for (int i = trajectory->stateTrajectory.size() - 1; i >= 0; --i) {
        for (int j = i - 2; j >= 0; --j) {
            if (j < i - 1) {
                if (distanceFunction_(trajectory->stateTrajectory[i],
                                      trajectory->stateTrajectory[j]) <= planningRange_) {
                    unsigned int k = j + 1;
                    while (k != i) {
                        markForDelete.push_back(k);
                        k++;
                    }

                    i = j;
                }
            }
        }


    }

    for (int i = markForDelete.size() - 1; i >= 0; --i) {
        trajectory->stateTrajectory.erase(trajectory->stateTrajectory.begin() + markForDelete[i]);

    }
}

const RobotStateSharedPtr RRTConnect::interpolate_(const RobotStateSharedPtr& state1,
        const RobotStateSharedPtr& state2,
        const FloatType& t) const
{
    return robotEnvironment_->getRobot()->getStateSpace()->interpolate(state1, state2, t);
}

}
