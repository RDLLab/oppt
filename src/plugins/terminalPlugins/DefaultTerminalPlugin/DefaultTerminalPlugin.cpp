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
#include "oppt/plugin/Plugin.hpp"
#include "DefaultTerminalOptions.hpp"
#include "GoalParser.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

namespace oppt
{
class DefaultTerminalPlugin: public TerminalPlugin
{
public :
    DefaultTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~DefaultTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<DefaultTerminalOptions>(optionsFile);        
        makeGoal();
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        auto options = static_cast<DefaultTerminalOptions*>(options_.get());
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        validityReport->isValid = true;
        validityReport->satisfiesConstraints = true;
        validityReport->collided = false;

        validityReport->satisfiesConstraints =
            robotEnvironment_->getRobot()->getStateSpace()->insideStateLimits(propagationResult->nextState);
        if (!validityReport->satisfiesConstraints)          
            validityReport->isValid = false;            
        
        if (!options->allowCollisions) {
            if (propagationResult->collisionReport) {
                validityReport->collided = propagationResult->collisionReport->collides;
            } else {
                oppt::CollisionReportSharedPtr collisionReport =
                    robotEnvironment_->getRobot()->makeDiscreteCollisionReport(propagationResult->nextState);
                validityReport->collided = collisionReport->collides;
            }

            if (validityReport->collided) {                
                validityReport->isValid = false;
            }

        }

        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
        auto options = static_cast<DefaultTerminalOptions*>(options_.get());        
        if (!isValid(propagationResult)->isValid)             
            return true;
        
        return satisfiesGoal(propagationResult->nextState);
    }

private:
    oppt::GoalSharedPtr goal_;

private:
    bool satisfiesGoal(const RobotStateSharedPtr& state) const {
        auto options = static_cast<DefaultTerminalOptions*>(options_.get());
        RobotStateSharedPtr denormalizedState =
            robotEnvironment_->getRobot()->getStateSpace()->denormalizeState(state);
        VectorFloat stateVec = denormalizedState->as<VectorState>()->asVector();
        GazeboWorldStatePtr worldState = state->getGazeboWorldState();
        geometric::Pose goalLinkPose = robotEnvironment_->getGazeboInterface()->getLinkPose(stateVec, worldState, options->goalLink);        
        VectorFloat goalLinkPosition(3, 0.0);
        if (options->goalLinkPoint.size() == 3) {
            geometric::Pose goalPointPoseFrame;
            goalPointPoseFrame.position[0] = options->goalLinkPoint[0];
            goalPointPoseFrame.position[1] = options->goalLinkPoint[1];
            goalPointPoseFrame.position[2] = options->goalLinkPoint[2];            
            goalLinkPose = geometric::Pose(goalPointPoseFrame.toGZPose() + goalLinkPose.toGZPose());
        }
        
        return static_cast<oppt::SphereGoal*>(goal_.get())->isSatisfied(toStdVec<FloatType>(goalLinkPose.position));
    }

    void makeGoal() {
        auto options = static_cast<DefaultTerminalOptions*>(options_.get());
        DefaultTerminalPluginGoalParser goalParser;
        VectorFloat goalArea = goalParser.parseGoalAreaFromFile(robotEnvironment_->getWorldFile());
        if (goalArea.size() != 4)
            ERROR("GoalArea has the wrong size");
        VectorFloat goalPosition( {goalArea[0], goalArea[1], goalArea[2]});
        FloatType goalRadius = goalArea[3];
        goal_ = std::make_shared<oppt::SphereGoal>(goalPosition, goalRadius);
        VectorString robotLinksInWorld = robotEnvironment_->getGazeboInterface()->getRobotLinkNames();
        if (!contains(robotLinksInWorld, options->goalLink))
            ERROR("link '" + options->goalLink + "' not defined in your robot model");
        if (!(options->goalLinkPoint.size() == 3))
            ERROR("The vector for the goalLinkPoint in your configuration file has the wrong number of dimensions. Must be 3");
    }

};

OPPT_REGISTER_TERMINAL_PLUGIN(DefaultTerminalPlugin)

}




