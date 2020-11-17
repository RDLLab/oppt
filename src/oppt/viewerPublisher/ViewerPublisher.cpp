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
#include "ViewerPublisher.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <stdexcept>
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/global.hpp"

namespace oppt
{

ViewerPublisher::ViewerPublisher():
    ViewerBase(),
    environmentMtx_(),
    baseFrame_(""),
    sdfParser_(),
    foundROSEnvironment_(false)
{
    int argc = 0;
    char** argv;
    char const* tmp = std::getenv("ROS_MASTER_URI");
    if (tmp) {
        std::string rosIP = std::getenv("ROS_MASTER_URI");
        ros::init(argc, argv, "ViewerPublisher", ros::init_options::AnonymousName);
        rosInitCalled_ = true;
        if (ros::master::check()) {
            // First check if the viewer is already being used by another process
            // If that's the case, don't continue initializing
            if (ros::param::has("/viewerOccupied")) {
                bool viewerOccupied = false;
                ros::param::get("/viewerOccupied", viewerOccupied);
                if (viewerOccupied) {
                    WARNING("Viewer is already occupied by another process. Visualization for this process is disabled.");
                    return;
                }
            }            

            foundROSEnvironment_ = true;
            ros::NodeHandle node;
            auto handler = [](int signal) {
                throw std::runtime_error("signal " + std::to_string(signal));
            };
            signal(SIGINT, handler);
            //signal(SIGINT, signalHandler);
            registerViewerPublisher_ =
                node.advertise<std_msgs::String>("registerViewerPublisher", 0, true);
            setFixedFramePublisher_ =
                node.advertise<std_msgs::String>("setFixedFrameTopic", 0, true);
            updateMarkersPublisher_ =
                node.advertise<visualization_msgs::MarkerArray>("viewerTopic", 100, true);
            resetFramePublisher_ =
                node.advertise<std_msgs::Bool>("resetFrameTopic", 100, true);
            setFrameRatePublisher_ = 
                node.advertise<std_msgs::Int16>("setFrameRateTopic", 100, true);


            std::string serviceName = ros::this_node::getName() + "/alive_service";
            aliveService_ =
                std::unique_ptr<ros::ServiceServer>(new ros::ServiceServer(node.advertiseService(serviceName, &ViewerPublisher::aliveCallback, this)));

            displayAddedSub_ = node.subscribe("displayAddedTopic",
                                              100,
                                              &ViewerPublisher::waitForDisplayAddedMessage,
                                              this);

            std_msgs::String viewerPublisherName;
            viewerPublisherName.data = ros::this_node::getName();
            registerViewerPublisher_.publish(viewerPublisherName);
        } else {
            WARNING("ROS master at '" + rosIP + "'' couldn't be reached. Visualization is disabled");
        }
    } else {
        WARNING("ROS_MASTER_URI environment variable not set. Visualization is disabled");
    }
}

ViewerPublisher::~ViewerPublisher()
{
    //displayAddedSub_.shutdown();
    if (foundROSEnvironment_) {
        setFixedFramePublisher_.shutdown();
        resetFramePublisher_.shutdown();
        displayAddedSub_.shutdown();
        registerViewerPublisher_.shutdown();
    }

    if (rosInitCalled_)
        ros::shutdown();
}

bool ViewerPublisher::aliveCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    response.success = true;
    return true;
}

bool ViewerPublisher::setupViewer(const std::string &robotName,
                                  const std::string &environmentFile, 
                                  const unsigned int &frameRate)
{
    if (foundROSEnvironment_) {
        environmentMarkers_ = sdfParser_.parseFromFile(environmentFile, robotName);
        robotMarkers_ = std::make_shared<SDFMarkers>();
        baseFrame_ = "world";
        reset();

        std_msgs::Int16 frameRateMsg;
        frameRateMsg.data = frameRate;
        setFrameRatePublisher_.publish(frameRateMsg);
        return true;
    }

    return false;
}

void ViewerPublisher::waitForDisplayAddedMessage(const std_msgs::BoolConstPtr& msg)
{
    displayAdded_ = true;
}

void ViewerPublisher::reset()
{
    if (foundROSEnvironment_) {
        displayAdded_ = false;
        std_msgs::Bool resetMsg;
        resetFramePublisher_.publish(resetMsg);
        ros::spinOnce();
        FloatType t0 = oppt::clock_ms();
        //boost::timer t0;
        while (!displayAdded_) {
            ros::spinOnce();
            if ((oppt::clock_ms() - t0) / 1000.0 > 0.3)
                break;
        }
        std_msgs::String setFixedFrameMsg;
        setFixedFrameMsg.data = baseFrame_;
        setFixedFramePublisher_.publish(setFixedFrameMsg);
        ros::spinOnce();
    }
}

void ViewerPublisher::updateFromEnvironmentInfo(const EnvironmentInfoSharedPtr& environmentInfo, bool forceReset)
{
    if (foundROSEnvironment_) {
        updateFromEnvironmentInfoImpl(environmentInfo, forceReset);
    }
}

void ViewerPublisher::updateFromEnvironmentInfoImpl(const EnvironmentInfoSharedPtr& environmentInfo, const bool& forceReset)
{
    boost::recursive_mutex::scoped_lock scoped_lock(environmentMtx_);
    if (forceReset) {
        reset();
    }


    VectorBodyPtr bodies = environmentInfo->getScene()->getBodies();
    removeMarkersWithName(environmentMarkers_->markerNames, environmentMarkers_);
    VectorGeometryPtr geometries;
    for (size_t i = 0; i != bodies.size(); ++i) {
        VectorGeometryPtr visualGeometries = bodies[i]->getVisualGeometries();
        geometries.insert(geometries.end(), visualGeometries.begin(), visualGeometries.end());        
    }

    VectorMarkers currentMarkers;
    VectorString currentMarkerNames(geometries.size());
    SDFMarkersSharedPtr geometryMarkers = sdfParser_.geometriesToMarkers(geometries);
    for (size_t i = 0; i != geometries.size(); ++i) {
        currentMarkers.push_back(geometryMarkers->markers[i]);
        currentMarkerNames[i] = geometryMarkers->markerNames[i];
        currentMarkers[i]->id = i;
    }

    environmentMarkers_->markers = currentMarkers;
    environmentMarkers_->markerNames = currentMarkerNames;
    //if (forceReset)
    //  usleep(50000);
    publishMarkers();
}

void ViewerPublisher::publishMarkers()
{    
    visualization_msgs::MarkerArray updateMarkersMsgs;
    for (size_t i = 0; i != robotMarkers_->markers.size(); ++i) {
        robotMarkers_->markers[i]->id = i;
        updateMarkersMsgs.markers.push_back(*(robotMarkers_->markers[i].get()));
        updateMarkersMsgs.markers[i].header.frame_id = baseFrame_;
    }    

    updateMarkersPublisher_.publish(updateMarkersMsgs);
}

bool ViewerPublisher::drawGeometries(VectorGeometryPtr& geometries,
                                     std::vector<std::vector<geometric::Pose>>& poses,
                                     const FloatType& particleOpacity,
                                     const bool& keepParticles,
                                     const bool& deleteExistingParticles)
{
    if (foundROSEnvironment_) {
        std_msgs::String setFixedFrameMsg;
        setFixedFrameMsg.data = baseFrame_;
        setFixedFramePublisher_.publish(setFixedFrameMsg);
        if (poses.size() > 0 || deleteExistingParticles)
            removeParticles(deleteExistingParticles);
        SDFMarkersSharedPtr geometryMarkers = sdfParser_.geometriesToMarkers(geometries);

        // Set the pose of the particles
        for (size_t i = 0; i != poses.size(); ++i) {
            for (size_t j = 0; j != poses[i].size(); ++j) {
                MarkerSharedPtr marker = sdfParser_.geometryToMarker(geometries[j], poses[i][j], particleOpacity);
                geometryMarkers->markers.push_back(marker);
                geometryMarkers->markerNames.push_back(geometryMarkers->markerNames[j] + "_particle_" + std::to_string(i));
            }
        }


        boost::recursive_mutex::scoped_lock scoped_lock(environmentMtx_);
        robotMarkers_ = geometryMarkers;
        publishMarkers();
        return true;

        //updateMarkersPublisher_.publish(updateMarkersMsgs);
        //ros::spinOnce();
    }

    return false;
}

void ViewerPublisher::displayText(const std::string& text)
{
    if (foundROSEnvironment_) {
        SDFMarkersSharedPtr textMarkers = sdfParser_.textToMarkers(text);
        visualization_msgs::MarkerArray updateMarkersMsgs;
        for (auto & textMarker : textMarkers->markers) {
            updateMarkersMsgs.markers.push_back(*(textMarker.get()));
            updateMarkersMsgs.markers[updateMarkersMsgs.markers.size() - 1].header.frame_id = baseFrame_;
        }

        LOGGING("Publish text marker");
        updateMarkersPublisher_.publish(updateMarkersMsgs);
        ros::spinOnce();
    }
}

void ViewerPublisher::removeParticles(const bool& publish)
{
    if (foundROSEnvironment_) {
        boost::recursive_mutex::scoped_lock scoped_lock(environmentMtx_);
        VectorMarkers newMarkers;
        VectorString newMarkerNames;
        visualization_msgs::MarkerArray markersToDelete;
        for (size_t i = 0; i != robotMarkers_->markerNames.size(); ++i) {
            if (robotMarkers_->markerNames[i].find("particle") != std::string::npos) {
                robotMarkers_->markers[i]->header.stamp = ros::Time::now();
                robotMarkers_->markers[i]->header.frame_id = baseFrame_;
                robotMarkers_->markers[i]->action = visualization_msgs::Marker::DELETE;
                markersToDelete.markers.push_back(*(robotMarkers_->markers[i].get()));
            } else {
                newMarkers.push_back(robotMarkers_->markers[i]);
                newMarkerNames.push_back(robotMarkers_->markerNames[i]);
            }
        }

        if (markersToDelete.markers.size() > 0 && publish) {
            updateMarkersPublisher_.publish(markersToDelete);
            ros::spinOnce();
        }

        robotMarkers_->markers = newMarkers;
        robotMarkers_->markerNames = newMarkerNames;
    }
}

void ViewerPublisher::removeMarkersWithName(const VectorString& names, SDFMarkersSharedPtr& markers, const bool& publish)
{
    if (foundROSEnvironment_) {
        boost::recursive_mutex::scoped_lock scoped_lock(environmentMtx_);
        VectorMarkers newMarkers;
        VectorString newMarkerNames;
        visualization_msgs::MarkerArray markersToDelete;
        std::string topic = "the_markers";
        bool toDelete;
        for (size_t i = 0; i < markers->markerNames.size(); i++) {
            toDelete = false;
            for (auto & name : names) {
                if (markers->markerNames[i] == name) {
                    markers->markers[i]->header.stamp = ros::Time::now();
                    markers->markers[i]->header.frame_id = baseFrame_;
                    markers->markers[i]->action = visualization_msgs::Marker::DELETE;
                    markersToDelete.markers.push_back(*(markers->markers[i].get()));
                    toDelete = true;
                    break;
                }
            }

            if (!toDelete) {
                newMarkers.push_back(markers->markers[i]);
                newMarkerNames.push_back(markers->markerNames[i]);
            }
        }

        for (size_t i = 0; i < newMarkers.size(); i++) {
            newMarkers[i]->id = i;
        }

        if (markersToDelete.markers.size() > 0) {
            if (publish) {
                updateMarkersPublisher_.publish(markersToDelete);
                ros::spinOnce();
            }
            markers->markers = newMarkers;
            markers->markerNames = newMarkerNames;
        }
    }
}

bool ViewerPublisher::viewerRunning() const
{
    return foundROSEnvironment_;
    //return ros::master::check();
}

}




