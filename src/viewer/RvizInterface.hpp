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
#ifndef _RVIZ_INTERFACE_HPP_
#define _RVIZ_INTERFACE_HPP_
#include <QWidget>
#include <QApplication>
#include <QVBoxLayout>
#include <memory>
#include <thread>
#include <ros/ros.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include "CommandTypes.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <visualization_msgs/MarkerArray.h>

namespace oppt
{

typedef std::queue<DisplayCommand*> CommandQueue;

typedef std::shared_ptr<CommandQueue> CommandQueuePtr;

class DisplayManager
{
public:
    DisplayManager(rviz::RenderPanel* renderPanel);

    ~DisplayManager();

    rviz::Display* addDisplay(AddDisplayCommand* addDisplayCommand);

    bool removeDisplay(std::string name);

    void setFrameRate(const unsigned int &frameRate);

    rviz::VisualizationManager* getManager() const;

private:
    std::unique_ptr<rviz::VisualizationManager> manager_;
    
    ros::Publisher displayAddedPub_;
    
    //std::map<std::string, std::unique_ptr<rviz::Display>> displays_;    

};

class RvizWidget: public QWidget
{
public:
    RvizWidget(QWidget* parent = 0);

    bool init(rviz::RenderPanel* renderPanel);

private:
    std::unique_ptr<QGridLayout> mainLayout_;

};


class RvizInterface: public QObject
{
public:
    RvizInterface();    

    void init();

    void initSubscriber();

private:

    void run(CommandQueuePtr dQueue);

    bool initializeRos();

    bool startRoscore();    

    void receiveCommands(const visualization_msgs::MarkerArrayConstPtr& msg);
    
    void receiveBaseFrameCommand(const std_msgs::StringConstPtr &msg);
    
    void receiveGrabScreenCommand(const std_msgs::StringConstPtr &msg);
    
    void receiveShowWorldFrameCommand(const std_msgs::BoolConstPtr& msg);

    void receiveSetFrameRateCommand(const std_msgs::Int16ConstPtr &msg);

    void addToQueue(DisplayCommand* command);

    void processQueue(CommandQueuePtr& dQueue);
    
    void reset(const std_msgs::BoolConstPtr &b);

    void registerViewerPublisherCallback(const std_msgs::String::ConstPtr& msg);

    void spin();

private:   
    std::string fixedFrame_;

    std::unique_ptr<rviz::RenderPanel> renderPanel_;

    std::unique_ptr<RvizWidget> rvizWidget_;

    std::unique_ptr<DisplayManager> displayManager_;

    CommandQueuePtr dQueue_;
    
    std::unique_ptr<std::thread> viewerThread_;
    
    std::unique_ptr<std::thread> roscoreThread_;

    std::unique_ptr<std::thread> spinThread_;

    std::unique_ptr<ros::Subscriber> sub_ = nullptr;
    std::unique_ptr<ros::Subscriber> sub2_ = nullptr;
    std::unique_ptr<ros::Subscriber> sub3_ = nullptr;
    std::unique_ptr<ros::Subscriber> sub4_ = nullptr;
    std::unique_ptr<ros::Subscriber> sub5_ = nullptr;
    std::unique_ptr<ros::Subscriber> sub6_ = nullptr;  

    std::unique_ptr<ros::Subscriber> registerViewerPublisherSubscriber_ = nullptr;

    std::unique_ptr<ros::ServiceClient> aliveServiceClient_ = nullptr;

    bool destroy_ = false;

    boost::mutex mtx_;
    
    ros::Publisher vizPub_;

    std::string registeredViewerPublisher_ = "";

};

}

#endif
