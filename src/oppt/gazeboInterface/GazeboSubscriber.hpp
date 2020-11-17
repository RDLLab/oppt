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
#ifndef _GAZEBO_SUBSCRIBER_HPP_
#define _GAZEBO_SUBSCRIBER_HPP_
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include "oppt/opptCore/core.hpp"

using namespace gazebo;
using namespace transport;

namespace oppt
{
    
class GazeboInterface;
    
class GazeboSubscriber
{
    typedef std::function<void(const std::string&)> RemoveBodyCallback;
    
    typedef std::function<void(const std::string&, const geometric::Pose&)> PoseChangedCallback;
public:
    GazeboSubscriber(GazeboInterface *gazeboInterface, const std::string &worldName);
    
    virtual ~GazeboSubscriber();
    
    void setRemoveBodyCallback(RemoveBodyCallback &removeBodyCallback);
    
    void setBodyPoseChangedCallback(PoseChangedCallback &poseChangedCallback);
    
    void suppressProcessing(const bool &suppress);

    void unsubscribe();
    
private:
    void onRequest(ConstRequestPtr &_msg);
    
    void onModify(ConstModelPtr &_msg);
    
private:
    GazeboInterface *gazeboInterface_ = nullptr;
    
    RemoveBodyCallback removeBodyCallback_ = nullptr;
    
    PoseChangedCallback poseChangedCallback_ = nullptr;
    
    std::string worldName_;
    
    gazebo::transport::NodePtr node_;
    
    SubscriberPtr requestSubscriber_;
    
    SubscriberPtr modifySubscriber_;
    
    bool processingSuppressed_ = false;

    bool worldChangesIgnored_ = false;

};
}

#endif
