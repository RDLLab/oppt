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
#ifndef _VIEWER_PUBLISHER_HPP_
#define _VIEWER_PUBLISHER_HPP_
#include <ros/ros.h>
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/Viewer.hpp"
#include "SDFParser.hpp"
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/recursive_mutex.hpp>
#include <bondcpp/bond.h>

namespace oppt
{

class ViewerPublisher: public ViewerBase
{
public:
    using ViewerBase::ViewerBase;
    _NO_COPY_OR_MOVE(ViewerPublisher)
    /**
     * Default constructor
     */
    ViewerPublisher();

    /**
     * Default destructor
     */
    virtual ~ViewerPublisher();

    /**
     * Setup the viewer publisher to display the provided model and environment
     */
    bool setupViewer(const std::string &robotName,
                     const std::string &environmentFile,
                     const unsigned int &frameRate);

    bool drawGeometries(VectorGeometryPtr& geometries,
                        std::vector<std::vector<geometric::Pose>>& poses,
                        const FloatType &particleOpacity=1.0,
                        const bool& keepParticles = false,
                        const bool& deleteExistingParticles = false);

    void updateFromEnvironmentInfo(const EnvironmentInfoSharedPtr& environmentInfo, bool forceReset = false);

    void displayText(const std::string& text);

    /**
     * Determine if the viewer is running
     */
    virtual bool viewerRunning() const override;

private:
    void removeMarkersWithName(const VectorString& names, SDFMarkersSharedPtr& markers, const bool& publish = false);

    void updateFromEnvironmentInfoImpl(const EnvironmentInfoSharedPtr& environmentInfo, const bool& forceReset);

    void publishMarkers();

    void reset();

    void waitForDisplayAddedMessage(const std_msgs::BoolConstPtr& msg);

    void removeParticles(const bool &publish);

    bool aliveCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

private:
    bool destroy_ = false;

    SDFMarkersSharedPtr environmentMarkers_;

    SDFMarkersSharedPtr robotMarkers_;

    SDFMarkersSharedPtr frameMarkers_;

    boost::recursive_mutex environmentMtx_;

    std::string baseFrame_;

    SDFParser sdfParser_;

    bool displayAdded_ = true;

    ros::Publisher updateMarkersPublisher_;

    ros::Publisher setFixedFramePublisher_;

    ros::Publisher resetFramePublisher_;

    ros::Publisher setFrameRatePublisher_;

    ros::Publisher registerViewerPublisher_;

    ros::Subscriber displayAddedSub_;

    bool foundROSEnvironment_ = false;

    bool rosInitCalled_ = false;

    std::unique_ptr<ros::ServiceServer> aliveService_ = nullptr;

};
}

#endif
