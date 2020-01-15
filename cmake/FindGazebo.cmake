# - Find Gazebo
# Find Gazebo and it's root include path
#
#   gazebo_FOUND       - True if Gazebo found.
#   gazebo_INCLUDE_DIRS - where to find Gazebo headersz
#   gazebo_LIBRARIES   - Gazebo library.
#

find_package(gazebo REQUIRED)
if(NOT gazebo_FOUND)
   message(FATAL_ERROR "Gazebo could not be found")
endif()

if (gazebo_VERSION VERSION_LESS "8.0")
 set(GZ_GT_7 False)
else()
 set(GZ_GT_7 True)
 add_definitions(-DGZ_GT_7)
endif()

FIND_PATH( GAZEBO_INCLUDE_PATH "gazebo.hh"
           PATH_SUFFIXES "gazebo-${GAZEBO_MAJOR_VERSION}/gazebo" )
if (NOT GAZEBO_INCLUDE_PATH)
    message(FATAL_ERROR "Gazebo header could not be found")
endif()
list(APPEND GAZEBO_INCLUDE_DIRS "${GAZEBO_INCLUDE_PATH}")
message("-- Found Gazebo ${gazebo_VERSION}")
           

