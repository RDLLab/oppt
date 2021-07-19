#ifndef GIM_BOXPRUNING_H_INCLUDED
#define GIM_BOXPRUNING_H_INCLUDED

/*! \file gim_boxpruning.h
\author Francisco Le�n
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "GIMPACT/gim_radixsort.h"
#include "GIMPACT/gim_geometry.h"

/*! \defgroup BOX_PRUNNING

\brief
Tools for find overlapping objects on a scenary. These functions sort boxes for faster collisioin queries, using radix sort or quick sort as convenience. See \ref SORTING .
<ul>
<li> For using these collision routines, you must create a \ref GIM_AABB_SET by using this function : \ref gim_aabbset_alloc.
<li> The  GIM_AABB_SET objects must be updated on their boxes on each query, and they must be update by calling \ref gim_aabbset_update
<li> Before calling collision functions, you must create a pair set with \ref GIM_CREATE_PAIR_SET
<li> For finding collision pairs on a scene (common space for objects), call \ref gim_aabbset_self_intersections
<li> For finding collision pairs between two box sets , call \ref gim_aabbset_box_collision
<li> After using collision routines, you must destroy the pairset with \ref GIM_DESTROY_PAIR_SET
<li> When the box set is no longer used, you must destroy it by calling \ref gim_aabbset_destroy
</ul>
*/
//! @{
//! Overlapping pair
struct GIM_PAIR
{
    GUINT32 m_index1;
    GUINT32 m_index2;
};
//typedef struct _GIM_PAIR GIM_PAIR;

//! Box container
struct GIM_AABB_SET
{
    GUINT32 m_count;
    aabb3f m_global_bound;//!< Global calculated bound of all boxes
    aabb3f * m_boxes;
    GUINT32 * m_maxcoords;//!<Upper corners of the boxes, in integer representation
    GIM_RSORT_TOKEN * m_sorted_mincoords;//!< sorted min coords (lower corners), with their coord value as the m_key and m_value as the box index
    char m_shared;//!< if m_shared == 0 then the memory is allocated and the set must be destroyed, else the pointers are shared and the set should't be destroyed
};
//typedef  struct _GIM_AABB_SET GIM_AABB_SET;

//! Function for creating  an overlapping pair set
#define GIM_CREATE_PAIR_SET(dynarray) GIM_DYNARRAY_CREATE(GIM_PAIR,dynarray,G_ARRAY_GROW_SIZE)
//! Function for destroying an overlapping pair set
#define GIM_DESTROY_PAIR_SET(dynarray) GIM_DYNARRAY_DESTROY(dynarray)

//! Allocate memory for all aabb set.
void gim_aabbset_alloc(GIM_AABB_SET * aabbset, GUINT32 count);

//! Destroys the aabb set.
void gim_aabbset_destroy(GIM_AABB_SET * aabbset);

//! Calcs the global bound only
/*!
\pre aabbset must be allocated. And the boxes must be already set.
*/
void gim_aabbset_calc_global_bound(GIM_AABB_SET * aabbset);

//! Sorts the boxes for box prunning.
/*!
1) find the integer representation of the aabb coords
2) Sorts the min coords
3) Calcs the global bound
\pre aabbset must be allocated. And the boxes must be already set.
\param aabbset
\param calc_global_bound If 1 , calcs the global bound
\post If aabbset->m_sorted_mincoords == 0, then it allocs the sorted coordinates
*/
void gim_aabbset_sort(GIM_AABB_SET * aabbset, char calc_global_bound);

//! log(N) Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated and sorted, the boxes must be already set.
\param aabbset Must be sorted. Global bound isn't required
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_sorted(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs);

//! NxN Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated, the boxes must be already set.
\param aabbset Global bound isn't required. Doen't need to be sorted.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_brute_force(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs);

//! log(N) Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_sorted(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs);

//! NxM Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_brute_force(GIM_AABB_SET * aabbset1,GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs);


/*
                Brute-Force Vs Sorted pruning
Different approaches must be applied when colliding sets with different number of
elements. When sets have less of 100 boxes, is often better to apply force brute
approach instead of sorted methods, because at lowlevel bruteforce routines gives
better perormance and consumes less resources, due of their simplicity.
But when sets are larger, the complexiity of bruteforce increases exponencially.
In the case of large sets, sorted approach is applied. So GIMPACT has the following
strategies:

On Sorting sets:
!) When sets have more of 140 boxes, the boxes are sorted by its coded min coord
and the global box is calculated. But when sets are smaller (less of 140 boxes),
Is convenient to apply brute force approach.

*******************************************************************************/

//! Constant for apply approaches between brute force and sorted pruning on bipartite queries
#define GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES 600
//! Constant for apply approaches between brute force and sorted pruning for box collision
#define GIM_MIN_SORTED_PRUNING_BOXES 140


//Use these functions for general initialization

//! Initalizes the set. Sort Boxes if needed.
/*!
\pre aabbset must be allocated. And the boxes must be already set.
\post If the set has less of GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES boxes, only calcs the global box,
 else it Sorts the entire set( Only applicable for large sets)
*/
void gim_aabbset_update(GIM_AABB_SET * aabbset);

///Use these functions for general collision

//! Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
This function sorts the set and then it calls to gim_aabbset_self_intersections_brute_force or gim_aabbset_self_intersections_sorted. This is an example of how to use this function:
\code
//Create contact list
GDYNAMIC_ARRAY collision_pairs;
GIM_CREATE_PAIR_SET(collision_pairs);
//Do collision
gim_aabbset_self_intersections(&aabbset,&collision_pairs);
if(collision_pairs.m_size==0)
{
    GIM_DYNARRAY_DESTROY(collision_pairs);//
    return; //no collisioin
}

//pair pointer
GIM_PAIR *pairs = GIM_DYNARRAY_POINTER(GIM_PAIR,collision_pairs);
GUINT i, ti1,ti2;
for (i=0;i<collision_pairs.m_size; i++)
{
    ti1 = pairs[i].m_index1;
    ti2 = pairs[i].m_index2;
    //Do something with the pairs
    ....
    ....
    ...

}
//Terminate
GIM_DYNARRAY_DESTROY(dummycontacts);
GIM_DYNARRAY_DESTROY(collision_pairs);
\endcode
\param aabbset Set of boxes. Sorting isn't required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
\pre aabbset must be allocated and initialized.
\post If aabbset->m_count >= GIM_MIN_SORTED_PRUNING_BOXES, then it calls to gim_aabbset_sort and then to gim_aabbset_self_intersections_sorted. Global box won't be calculated.
*/
void gim_aabbset_self_intersections(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs);

//! Collides two sets. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and updated. See gim_aabbset_update.
\param aabbset1 Must be updated.
\param aabbset2 Must be updated.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs);

///Function for create Box collision result set

#define GIM_CREATE_BOXQUERY_LIST(dynarray) GIM_DYNARRAY_CREATE(GUINT32,dynarray,G_ARRAY_GROW_SIZE)

//! Finds intersections between a box and a set. Return the colliding boxes of the set
/*!
\pre aabbset must be allocated and initialized.
\param test_aabb Box for collision query
\param aabbset Set of boxes .Global bound is required.
\param collided Array of GUINT elements, indices of boxes. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_box_collision(aabb3f *test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided);

//! Finds intersections between a box and a set. Return the colliding boxes of the set
/*!
\pre aabbset must be allocated and initialized.
\param vorigin Origin point of ray.
\param vdir Direction vector of ray.
\param tmax Max distance param for ray.
\param aabbset Set of boxes .Global bound is required.
\param collided Array of GUINT elements, indices of boxes. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_ray_collision(vec3f vorigin,vec3f vdir, GREAL tmax, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided);


/*
For sorting, each box corner must be discretized to a 32 bit integer.
For this, we take the x and z coordinates from the box corner (a vector vec3f)
Then convert the (x,z) pair to an integer. For convenience, we choose an error
constant for converting the coordinates (0.05).
*******************************************************************************/

/**
 For fitting the coordinate to an integer, we need to constraint the range of its values. So each coord component (x, z) must lie between 0 and 65536.
 20 give us a 0.05 floating point error
*/
#define ERROR_AABB 20.0f

/**
An error of 0.05 allows to make coordinates up to 1638.0f and no less of -1638.0f.
So the maximum size of a room should be about 3276x3276 . Its dimensions must lie between  [-1638,1638.0f]
*/
#define MAX_AABB_SIZE 1638.0f

//! Converts a vector coordinate to an integer for box sorting
/*!
\param vx X component
\param vz Z component
\param uint_key a GUINT
*/
#define GIM_CONVERT_VEC3F_GUINT_XZ(vx,vz,uint_key)\
{\
    GUINT32 _z = ((GUINT32)(vz*ERROR_AABB))+32768;\
    uint_key = ((GUINT32)(vx*ERROR_AABB))+32768;\
    uint_key = (uint_key<<16) + _z;\
}\

//! Converts a vector coordinate to an integer for box sorting,rounding to the upper int
/*!
\param vx X component
\param vz Z component
\param uint_key a GUINT
*/
#define GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(vx,vz,uint_key)\
{\
    GUINT32 _z = ((GUINT32)ceilf(vz*ERROR_AABB))+32768;\
    uint_key = ((GUINT32)ceilf(vx*ERROR_AABB))+32768;\
    uint_key = (uint_key<<16) + _z;\
}\


//! Converts a vector coordinate to an integer for box sorting. Secure clamped
/*!
\param vx X component
\param vz Z component
\param uint_key a GUINT
*/
#define GIM_CONVERT_VEC3F_GUINT_XZ_CLAMPED(vx,vz,uint_key)\
{\
    GREAL _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);\
    GREAL _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);\
    GUINT32 _z = ((GUINT32)(_cz*ERROR_AABB))+32768;\
    uint_key = ((GUINT32)(_cx*ERROR_AABB))+32768;\
    uint_key = (uint_key<<16) + _z;\
}\

//! Converts a vector coordinate to an integer for box sorting. Secure clamped, rounded
/*!
\param vx X component
\param vz Z component
\param uint_key a GUINT
*/
#define GIM_CONVERT_VEC3F_GUINT_XZ_UPPER_CLAMPED(vx,vz,uint_key)\
{\
    GREAL _cx = CLAMP(vx,-MAX_AABB_SIZE,MAX_AABB_SIZE);\
    GREAL _cz = CLAMP(vz,-MAX_AABB_SIZE,MAX_AABB_SIZE);\
    GUINT32 _z = ((GUINT32)ceilf(_cz*ERROR_AABB))+32768;\
    uint_key = ((GUINT32)ceilf(_cx*ERROR_AABB))+32768;\
    uint_key = (uint_key<<16) + _z;\
}\

//! @}

#endif // GIM_BOXPRUNING_H_INCLUDED
