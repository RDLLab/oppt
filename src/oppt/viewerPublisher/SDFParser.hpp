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
#ifndef __SDF_PARSER_HPP__
#define __SDF_PARSER_HPP__
#include "Parser.hpp"
#include <sdf/parser.hh>
#include "oppt/opptCore/geometric/Box.hpp"
#include "oppt/opptCore/geometric/Sphere.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"
#include "oppt/opptCore/geometric/Cylinder.hpp"

namespace oppt
{

class SDFParser: public EnvironmentParser
{

public:
  using EnvironmentParser::EnvironmentParser;
  _NO_COPY_BUT_MOVE(SDFParser)

  SDFParser() = default;

  virtual ~SDFParser() = default; 

  SDFMarkersSharedPtr parseFromSDFString(std::string& sdfString, const std::string& robotName, const std::string &environmentFile = "") const;

  SDFMarkersSharedPtr parseFromFile(const std::string& environmentFile, const std::string& robotName) const override;

  MarkerSharedPtr boxToMarker(const geometric::Geometry *boxGeometry) const;

  MarkerSharedPtr sphereToMarker(const geometric::Geometry *sphereGeometry) const;

  MarkerSharedPtr cylinderToMarker(const geometric::Geometry *cylinderGeometry) const;

  MarkerSharedPtr meshToMarker(const geometric::Geometry *meshGeometry) const;

  MarkerSharedPtr geometryToMarker(const geometric::Geometry *geometry, const geometric::Pose& pose, const FloatType& opacity) const;

  SDFMarkersSharedPtr geometriesToMarkers(VectorGeometryPtr& geometries) const;

  SDFMarkersSharedPtr textToMarkers(const std::string& text) const;

private:
  geometry_msgs::Pose processPoseElement(sdf::ElementPtr& poseElement) const;

  bool processMaterialElement(std::shared_ptr<visualization_msgs::Marker>& marker,
                              sdf::ElementPtr& visualElement) const;

  bool processSphereElement(sdf::ElementPtr& sphereElement,
                            sdf::ElementPtr& poseElement,
                            sdf::ElementPtr& visualElement,
                            SDFMarkersSharedPtr& environmentMarkers) const;

  bool processCylinderElement(sdf::ElementPtr& cylinderElement,
                              sdf::ElementPtr& poseElement,
                              sdf::ElementPtr& visualElement,
                              SDFMarkersSharedPtr& environmentMarkers) const;

  bool processBoxElement(sdf::ElementPtr& boxElement,
                         sdf::ElementPtr& poseElement,
                         sdf::ElementPtr& visualElement,
                         SDFMarkersSharedPtr& environmentMarkers) const;

  bool processMeshElement(sdf::ElementPtr& meshElement,
                          sdf::ElementPtr& poseElement,
                          sdf::ElementPtr& visualElement,
                          SDFMarkersSharedPtr& environmentMarkers,
                          const std::string& environmentFile) const;

  int getMarkerId(SDFMarkersSharedPtr& environmentMarkers) const;
};

}

#endif
