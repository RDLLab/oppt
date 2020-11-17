SET(BOOST_MIN_VERSION "1.55.0")

include(FindPkgConfig)
include(GNUInstallDirs)

set(oppt_LIBRARY_DIRS "")
set(oppt_INCLUDE_TARGETS "")

############################# LOAD TINYXML #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindTinyXML.cmake)
if (NOT TINYXML_FOUND)
    message(FATAL_ERROR "TinyXML could not be found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${TINYXML_INCLUDE_DIRS}")

############################# LOAD LIBSPATIALINDEX #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindSpatialIndex.cmake)
if (NOT SPATIALINDEX_FOUND)
    message(FATAL_ERROR "libspatialindex could not be found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${SPATIALINDEX_INCLUDE_DIRS}")

############################# LOAD SDFORMAT #############################
set (SDFormat_VERSION 4.1.0)
find_package(SDFormat ${SDFormat_MIN_VERSION} REQUIRED)
if(NOT SDFormat_FOUND)   
   message(FATAL_ERROR "SDF could not be found")
endif()
list(APPEND oppt_INCLUDE_TARGETS "${SDFormat_INCLUDE_DIRS}")   

############################# LOAD EIGEN #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(EIGEN eigen3)
    if(NOT EIGEN_FOUND)
       message(FATAL_ERROR "EIGEN could not be found")
    endif()
    include_directories(${EIGEN_INCLUDE_DIRS})
    if (EIGEN_VERSION LESS 3.3.4)
    else()
       add_definitions(-DEIGEN_GT_3_3_4)
    endif()       
endif()
list(APPEND oppt_INCLUDE_TARGETS "${EIGEN_INCLUDE_DIRS}")

############################# LOAD ASSIMP #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindAssimp.cmake)
list(APPEND oppt_INCLUDE_TARGETS "${ASSIMP_INCLUDE_PATH}") 

############################# LOAD FCL #############################
if(PKG_CONFIG_FOUND)
    pkg_check_modules(FCL fcl)    
    if(NOT FCL_FOUND)
       message(FATAL_ERROR "FCL could not be found")
    endif()
    if (FCL_VERSION GREATER 0.4.0)
        add_definitions(-DFCL_GT_0_4)
    endif()       
endif()
list(APPEND oppt_INCLUDE_TARGETS "${FCL_INCLUDE_DIRS}")   

############################# LOAD ROS #############################
set(USE_RVIZ False)
if (BUILD_VIEWER)
find_package(catkin COMPONENTS roscpp rviz QUIET)
if (catkin_FOUND)
  message(STATUS "ROS and Rviz have been found compiling with viewer support")
  set(USE_RVIZ True)
  add_definitions(-DUSE_RVIZ)
  list(APPEND oppt_INCLUDE_TARGETS "${roscpp_INCLUDE_DIRS}")
else()
  message(STATUS "ROS and Rviz couldn't be found. Compiling without viewer support")   
endif()
endif()

############################# LOAD GAZEBO #############################
include(${CMAKE_CURRENT_LIST_DIR}/FindGazebo.cmake)
set(GZ_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
list(APPEND oppt_INCLUDE_TARGETS "${GAZEBO_INCLUDE_DIRS}")

############################# LOAD KDL PARSER #########################
set(SUPPORTS_IK True)

find_package(kdl_parser)
find_package(trac_ik_lib)
if(NOT kdl_parser_FOUND)
   message(STATUS "kdl_parser could not be found. Building without IK support")
   set(SUPPORTS_IK False)
endif()
if(NOT trac_ik_lib_FOUND)
   message(STATUS "trac_ik_lib could not be found. Building without IK support")
   set(SUPPORTS_IK False)
endif()
if (SUPPORTS_IK)  
   message(STATUS "Compiling with IK support")
   add_definitions(-DSUPPORTS_IK)
   list(APPEND oppt_INCLUDE_TARGETS "${kdl_parser_INCLUDE_DIRS}")
   list(APPEND oppt_INCLUDE_TARGETS "${trac_ik_lib_INCLUDE_DIRS}")
endif()


############################# LOAD BOOST #############################
find_package(Boost
             REQUIRED 
             system 
             thread             
             filesystem 
             serialization)
if (Boost_FOUND)    
    list(APPEND oppt_INCLUDE_TARGETS "${Boost_INCLUDE_DIRS}") 
endif ()
