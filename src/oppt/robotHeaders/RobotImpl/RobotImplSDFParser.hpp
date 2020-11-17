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
#ifndef __ROBOT_IMPL_SDF_PARSER_HPP__
#define __ROBOT_IMPL_SDF_PARSER_HPP__
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"
#include <sdf/parser.hh>

namespace oppt
{
class RobotImplSDFParser
{
public:
    RobotImplSDFParser();

    VectorGeometryUniquePtr parseVisualGeometries(const std::string& robotName, const std::string &environmentFile) const;

    VectorGeometryUniquePtr parseCollisionGeometries(const std::string& robotName, const std::string &environmentFile) const;

private:
    GeometryUniquePtr processBoxGeometry(sdf::ElementPtr boxGeometryElem,
                                         const std::string& visualName,
                                         const sdf::ElementPtr& visualElement) const;

    GeometryUniquePtr processMeshGeometry(sdf::ElementPtr meshGeometryElem,
                                          const std::string& visualName,
                                          const sdf::ElementPtr& visualElement) const;

    GeometryUniquePtr processSphereGeometry(sdf::ElementPtr sphereGeometryElem,
                                            const std::string& visualName,
                                            const sdf::ElementPtr& visualElement) const;

    GeometryUniquePtr processCylinderGeometry(sdf::ElementPtr cylinderGeometryElem,
            const std::string& visualName,
            const sdf::ElementPtr& visualElement) const;

    void processMaterialElement(const sdf::ElementPtr& visualElement, geometric::Geometry *geometryElem) const;



};

}

#endif
