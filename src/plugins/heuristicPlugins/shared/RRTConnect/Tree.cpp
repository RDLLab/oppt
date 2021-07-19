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
#include "Tree.hpp"
#include <limits>
#include "oppt/robotHeaders/RobotState.hpp"

namespace oppt
{

/** ------------------ Node ---------------*/
Node::Node(RRTTree* tree, const Node* parent, const VectorFloat& values):
    tree_(tree),
    children_(),
    parent_(parent),
    values_(values)
{

}

Node::~Node()
{
    children_.clear();
}

const Node* Node::getParent() const
{
    return parent_;
}


void Node::addChild(const size_t& idx)
{
    children_.push_back(idx);
}

void Node::setRobotState(const RobotStateSharedPtr& robotState)
{
    robotState_ = robotState;
}

const RobotStateSharedPtr Node::getRobotState() const
{
    return robotState_;
}

/** ----------------- Tree ----------------*/
RRTTree::RRTTree():
    nodes_()
{

}

RRTTree::~RRTTree()
{
    for (auto & node : nodes_) {
        if (node) {
            delete node;
        }
    }

    nodes_.clear();
}

void RRTTree::setDistanceFunction(DistanceFunction& distanceFunction)
{
    distanceFunction_ = distanceFunction;
}

const Node* RRTTree::allocNode(const Node* parent, const RobotStateSharedPtr& state)
{
    nodes_.push_back(new Node(this, 
                              parent, 
                              state->as<VectorState>()->asVector()));    
    nodes_[nodes_.size() - 1]->setRobotState(state);
    return nodes_[nodes_.size() - 1];
}


void RRTTree::reset()
{
    for (auto & node : nodes_) {
        if (node) {
            delete node;
        }
    }

    nodes_.clear();
}

size_t RRTTree::makeRoot(const RobotStateSharedPtr& state)
{    
    allocNode(nullptr, state);
    return nodes_.size() - 1;
}

const Node* RRTTree::nearestNeighbour(const RobotStateSharedPtr& q)
{
    FloatType smallestDist = std::numeric_limits<FloatType>::infinity();
    size_t smallestIdx = 0;
    FloatType dist = 0;
    VectorFloat qVec = q->as<VectorState>()->asVector();
    for (size_t i = 0; i != nodes_.size(); ++i) {
        dist = distanceFunction_(nodes_[i]->getRobotState(), 
                                 q);
        if (dist < smallestDist) {
            smallestDist = dist;
            smallestIdx = i;
        }
    }

    return nodes_[smallestIdx];
}

const FloatType RRTTree::size() const
{
    return nodes_.size();
}


}
