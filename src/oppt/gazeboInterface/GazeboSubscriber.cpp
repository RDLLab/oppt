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
#include "GazeboSubscriber.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

using namespace oppt;

GazeboSubscriber::GazeboSubscriber(GazeboInterface* gazeboInterface, const std::string& worldName):
    gazeboInterface_(gazeboInterface),
    worldName_(worldName),
    node_(gazebo::transport::NodePtr(new gazebo::transport::Node())),
    worldChangesIgnored_(true)
{
    node_->Init("GazeboSubscriber");
    std::string topicStringRequest = "/gazebo/" + worldName_ + "/request";
    std::string topicStringModify = "/gazebo/" + worldName_ + "/model/modify";
    requestSubscriber_ = node_->Subscribe(topicStringRequest, &GazeboSubscriber::onRequest, this, true);
    modifySubscriber_ = node_->Subscribe(topicStringModify, &GazeboSubscriber::onModify, this, true);
}

GazeboSubscriber::~GazeboSubscriber()
{
    processingSuppressed_ = true;
    requestSubscriber_.reset();
    modifySubscriber_.reset();
    node_.reset();
}

void GazeboSubscriber::unsubscribe() {
    worldChangesIgnored_ = true;
}


void GazeboSubscriber::setRemoveBodyCallback(RemoveBodyCallback& removeBodyCallback)
{
    removeBodyCallback_ = removeBodyCallback;
}

void GazeboSubscriber::setBodyPoseChangedCallback(PoseChangedCallback& poseChangedCallback)
{
    poseChangedCallback_ = poseChangedCallback;
}

void GazeboSubscriber::suppressProcessing(const bool& suppress)
{
    processingSuppressed_ = suppress;
}

void GazeboSubscriber::onRequest(ConstRequestPtr& msg)
{
    if (!processingSuppressed_ && removeBodyCallback_ && !worldChangesIgnored_) {
        if (msg->request() == "entity_delete") {
            //LOGGING("ENTITY DELETE: " + msg->data());
            auto data = msg->data();
            if (data.find("__COLLISION_VISUAL__") != std::string::npos) {
                VectorString elems;
                split(data, "::", elems);
                LOGGING("CALL REMOVE BODY CALLBACK");
                removeBodyCallback_(elems[0]);
            }
        }
    }
}

void GazeboSubscriber::onModify(ConstModelPtr& _msg)
{
    if (!processingSuppressed_ && poseChangedCallback_ && !worldChangesIgnored_) {
        if (_msg->has_pose()) {
            std::string name = _msg->name();
            LOGGING("ON MODIFY " + name);
            auto pose = _msg->pose();
            VectorFloat poseVec(7);

            geometric::Pose opptPose;
            opptPose.position = toEigenVec(VectorFloat{pose.position().x(), pose.position().y(), pose.position().z()});
            opptPose.orientation = Quaternionf(pose.orientation().w(), pose.orientation().x(), pose.orientation().y(), pose.orientation().z());
            gazeboInterface_->changeModelPose(name, opptPose);
            poseChangedCallback_(name, opptPose);
        }
    }
}
