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
#ifndef __ROBOT_IMPL_HPP__
#define __ROBOT_IMPL_HPP__
#include "oppt/robotHeaders/robot.hpp"
#include "RobotImplSerializer.hpp"
#include "RobotConfigOptions.hpp"

namespace oppt
{

class RobotImpl: public Robot
{
public:
    RobotImpl(const std::string &worldFile,
              const std::string &robotName,
              const std::string &configFile,
              const std::string &prefix,
              unsigned int& threadId);

    virtual bool init() override;

    virtual CollisionReportSharedPtr makeDiscreteCollisionReportDirty() const override;

    virtual void createRobotCollisionObjects(const oppt::RobotStateSharedPtr& state) const override;

    virtual unsigned int getDOF() const override;

    virtual void updateViewer(const oppt::RobotStateSharedPtr& state,
                              const std::vector<oppt::RobotStateSharedPtr>& particles,
                              const FloatType& particleOpacity = 1.0,
                              const bool& keepParticles = false,
                              const bool& deleteExistingParticles = false) override;

    virtual void setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction) override;

    virtual VectorString getCollisionCheckedLinks() const override;

protected:
    virtual void addBodyCallback(Body *const body) override;

    virtual void removeBodyCallback(std::string name) override;

    virtual void changeBodyPoseCallback(const std::string& name, const geometric::Pose& poseVec) override;

    VectorString collisionLinks_;

    std::map<std::string, GeometrySharedPtr> linkVisualGeometryMap_;

    unsigned int dof_;

    RobotConfigOptions robotConfigOptions_;

    std::string worldFile_;

protected:
    virtual void setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo) override;

    virtual void initEnvironmentCallbacks() override;

    virtual void registerOnAddBodyCallback(std::function<void(const std::string&)>& callback) override;

    virtual void registerOnRemoveBodyCallback(std::function<void(const std::string&)>& callback) override;

    virtual void registerOnPoseChangedCallback(std::function<void(const std::string&, const geometric::Pose&)>& callback) override;

    virtual void setCollisionsAllowed(const bool& collisionsAllowed) const override;

private:
    virtual bool makeActionSpace(ActionSpaceInfo& actionSpaceInfo) override;

    virtual bool makeStateSpace(StateSpaceInfo& stateSpaceInfo) override;

    virtual bool makeObservationSpace(ObservationSpaceInfo& observationSpaceInfo) override;

};

}

#endif
