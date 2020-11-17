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
#ifndef _VIEWER_HPP_
#define _VIEWER_HPP_

namespace oppt
{
    
/**
 * Base class for a viewer implementation
 */    
class ViewerBase
{
public:
    _NO_COPY_OR_MOVE(ViewerBase)
    /**
     * @brief Default constructor
     */
    ViewerBase()=default;
    
    /**
     * @brief Default destructor
     */
    virtual ~ViewerBase()=default;
    
    /**
     * @brief Determine if the viewer is running
     */
    virtual bool viewerRunning() const {
	return false;
    }
};
}

#endif
