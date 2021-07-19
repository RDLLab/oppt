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
#ifndef __ENVIRONMENT_INFO_HPP__
#define __ENVIRONMENT_INFO_HPP__
#include "typedefs.hpp"
#include "Body.hpp"
#include "Scene.hpp"

namespace oppt
{
class EnvironmentInfo
{
public:
    _NO_COPY_BUT_MOVE(EnvironmentInfo)
    
    EnvironmentInfo(const bool &interactive):
        scene_(),
        interactive_(interactive) {

    }

    EnvironmentInfoSharedPtr clone() const {
        EnvironmentInfoSharedPtr clonedEnvironmentInfo = std::make_shared<EnvironmentInfo>(interactive_);
        auto clonedScene = scene_->clone();
        clonedEnvironmentInfo->setScene(clonedScene);
        return clonedEnvironmentInfo;
    }

    void setScene(const oppt::SceneSharedPtr &planningScene) {
        scene_ = planningScene;
    }

    oppt::SceneSharedPtr getScene() const {
        return scene_;
    }

    const bool isInteractive() const {
        return interactive_;
    }

private:
    oppt::SceneSharedPtr scene_;

    bool interactive_;
};
}

#endif
