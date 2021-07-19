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
#ifndef __PARSER_HPP__
#define __PARSER_HPP__
#include "oppt/opptCore/core.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace oppt
{

typedef visualization_msgs::Marker Marker;
typedef std::shared_ptr<Marker> MarkerSharedPtr;
typedef std::vector<MarkerSharedPtr> VectorMarkers;

struct SDFMarkers {
    VectorMarkers markers;
    
    VectorString markerNames;
};

typedef std::shared_ptr<SDFMarkers> SDFMarkersSharedPtr;


class EnvironmentParser
{
public:
	_NO_COPY_BUT_MOVE(EnvironmentParser)
    EnvironmentParser() = default;

    virtual ~EnvironmentParser() = default;
    
    virtual SDFMarkersSharedPtr parseFromFile(const std::string &environmentFile, const std::string &robotFile) const = 0;

};

}

#endif
