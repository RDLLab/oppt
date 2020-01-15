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
#ifndef __ROBOT_IMPL_SERIALIZER_HPP__
#define __ROBOT_IMPL_SERIALIZER_HPP__
#include "oppt/robotHeaders/Serializer.hpp"

namespace oppt
{

class RobotImplSerializer: public VectorSerializer
{
public:
    RobotImplSerializer(const std::string &worldFile);
    
    //_NO_COPY_NO_MOVE(RobotImplSerializer);
    
    virtual RobotStateSharedPtr loadState(std::istream& is) const override;
    
    virtual RobotStateSharedPtr loadState(const std::string& input) const override;

    virtual OpptUserDataSharedPtr loadUserData(std::istream& is) const override;
    
    void setWorld(const WorldPtr &world);
    
    void setStateSpace(StateSpace *stateSpace);
    
private:
    std::string worldFile_;
    
    WorldPtr world_;
    
    StateSpace *stateSpace_ = nullptr;

};

}

#endif
