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
#ifndef __OPPT_CONSTANTS__
#define __OPPT_CONSTANTS__

namespace oppt
{

namespace Distributions
{ 
    typedef const unsigned int DistributionType;
    
    DistributionType MULTIVARIATE_NORMAL=0;
    
    DistributionType WEIGHTED_DISCRETE=1;
    
    DistributionType UNIFORM=1;

}

namespace geometric {
    typedef const unsigned int GeometryType;
    
    GeometryType BOX=0;
    
    GeometryType SPHERE=1;
    
    GeometryType MESH=2;
    
    GeometryType CYLINDER=3;
}

namespace SpaceType
{
    typedef const unsigned int StateSpaceType;
    
    StateSpaceType VECTORSPACE=0;
    
    StateSpaceType DISCRETE=1;
    
    StateSpaceType COMPOUND=2;

}

};

#endif
