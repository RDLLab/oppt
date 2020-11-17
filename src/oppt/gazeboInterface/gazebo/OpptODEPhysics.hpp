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
#ifndef _OPPT_ODE_PHYSICS_HPP_
#define _OPPT_ODE_PHYSICS_HPP_
#include <unordered_map>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/Joint.hh>
#include "OpptJoint.hpp"

namespace gazebo
{
namespace physics
{
    
typedef std::vector<std::pair<std::string, std::vector<FloatType>>> CumulativeAnglesVec;
   
class OpptODEPhysics: public ODEPhysics
{
public:
    OpptODEPhysics(WorldPtr _world);

    virtual ~OpptODEPhysics();
    
    virtual void UpdateCollision() override;

    virtual LinkPtr CreateLink(ModelPtr _parent) override;

    virtual JointPtr CreateJoint(const std::string& _type, ModelPtr _parent) override;
    
    void enableJoints(const bool &enable);
    
    void blockCollisionCheck(const bool &blockCollisionCheck);
    
    bool collionCheckBlocked() const;
    
    CumulativeAnglesVec getCumulativeAngles() const;
    
    void setCumulativeAngles(const CumulativeAnglesVec &cumulativeAngles);
    
    void makeJointNameMap();

private:
    std::map<std::string, dSpaceID> spaces_;

    std::unordered_map<Joint*, OpptJoint*> jointEntries_;
    
    std::unordered_map<std::string, OpptJoint*> jointNameMap_;
    
    bool blockCollisionCheck_ = false;
};

}
}

void RegisterOpptODEPhysics();

#endif
