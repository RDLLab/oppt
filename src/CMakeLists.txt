cmake_minimum_required (VERSION 2.6)
project (oppt)

cmake_policy(SET CMP0012 NEW)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DEIGEN_NO_DEBUG")

# RelWithDebInfo is the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)
if (NOT CMAKE_BUILD_TYPE MATCHES RELWITHDEBINFO AND NOT CMAKE_BUILD_TYPE MATCHES DEBUG AND NOT CMAKE_BUILD_TYPE MATCHES RELEASE)
   message(FATAL_ERROR "${CMAKE_BUILD_TYPE} is not a valid build type. Supported types are Debug, Release and RelWithDebInfo (default)")
endif()

option(BUILD_PLUGINS "Build the POMDP model plugins" ON)
option(BUILD_SOLVERS "Build the POMDP solvers" ON)
option(BUILD_VIEWER "Build the viewer" ON)

add_definitions(-DUSE_DOUBLE_PRECISION=true)

include(FindPkgConfig)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


set(ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DROOT_PATH=${ROOT_PATH})

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT_PATH}/../bin")
set (OPPT_CMAKE_DIR ${ROOT_PATH}/../cmake)

include (${OPPT_CMAKE_DIR}/SearchForStuff.cmake)

if(GZ_GT_10)
  set(REQUIRED_CXX_STANDARD 17)
else()
  set(REQUIRED_CXX_STANDARD 11)
endif()

configure_package_config_file("${OPPT_CMAKE_DIR}/opptMacros.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/opptMacros.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
)

include(${CMAKE_CURRENT_BINARY_DIR}/opptMacros.cmake)
include(${ROOT_PATH}/oppt/options/CMakeLists.txt)
include(${ROOT_PATH}/oppt/opptCore/CMakeLists.txt)
include(${ROOT_PATH}/oppt/plugin/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robotEnvironment/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robots/CMakeLists.txt)
include(${ROOT_PATH}/oppt/gazeboInterface/CMakeLists.txt)
include(${ROOT_PATH}/oppt/utils/CMakeLists.txt)
include(${ROOT_PATH}/oppt/filter/CMakeLists.txt)
include(${ROOT_PATH}/oppt/problemEnvironment/CMakeLists.txt)
include(${ROOT_PATH}/oppt/robotHeaders/CMakeLists.txt)
include(${ROOT_PATH}/oppt/solver/CMakeLists.txt)
include(${ROOT_PATH}/oppt/CMakeLists.txt)

if (USE_RVIZ AND BUILD_VIEWER)
  include(${ROOT_PATH}/oppt/viewerPublisher/CMakeLists.txt)
endif()

########################################

set(COMMON_SRCS
    ${THIRD_PARTY_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/oppt/global.cpp)
    
set(OPPT_SRC
    ${COMMON_SRCS}
    ${OPTIONS_SRC}
    ${ROBOT_PROBLEM_SRC}    
    ${ROBOT_ENVIRONMENT_SRC}
    ${GAZEBO_INTERFACE_SRC}    
    ${ROBOTS_SRC}    
    ${OPPT_UTILS_SRC}
    ${FILTER_SRC}
    ${VIEWER_PUBLISHER_SRC}    
    ${CHANGES_PARSER_SRC})
    
set(THIRD_PARTY_LIBS
    ${Boost_LIBRARIES}
    ${FCL_LIBRARIES}                         
    ${TCLAP_LIBRARIES}                        
    ${LIBSPATIALINDEX_LIBRARIES}    
    ${catkin_LIBRARIES}
    ${SDF_LIBRARIES}
    ${GAZEBO_LIBRARIES}          
    ${URDFDOM_LIBRARIES}    
    ${ASSIMP_LIBRARIES}
    ${TINYXML_LIBRARIES}
    ${SPATIALINDEX_LIBRARIES}  
    ${THIRD_PARTY_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${kdl_parser_LIBRARIES} 
    ${trac_ik_lib_LIBRARIES}    
    ${CMAKE_DL_LIBS})

# Add the OPPT library target
add_library( oppt SHARED ${OPPT_SRC} )
target_include_directories(oppt PUBLIC $<INSTALL_INTERFACE:include>;${oppt_INCLUDE_TARGETS})
target_link_libraries (oppt ${THIRD_PARTY_LIBS})
set_target_properties(oppt PROPERTIES CXX_STANDARD ${REQUIRED_CXX_STANDARD} CXX_STANDARD_REQUIRED YES)
install(TARGETS oppt
        EXPORT "opptTargets" 
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if (BUILD_PLUGINS)
    message(STATUS "Compiling OPPT plugins")
    add_subdirectory(${ROOT_PATH}/plugins/)
endif()

if (BUILD_SOLVERS)
    message(STATUS "Compiling solvers")
    add_subdirectory(${ROOT_PATH}/solvers/)
endif()
    
if(${USE_RVIZ} AND BUILD_VIEWER) 
    message(STATUS "Compiling OPPT viewer")   
    add_subdirectory(${ROOT_PATH}/viewer/)    
endif()

configure_package_config_file("${OPPT_CMAKE_DIR}/opptConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/opptConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

install(EXPORT opptTargets
  FILE "opptTargets.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/opptConfig.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt"
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/opptMacros.cmake"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/oppt")

install(FILES ${ROOT_PATH}/../setup.sh DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)
install(DIRECTORY ${ROOT_PATH}/../models DESTINATION ${CMAKE_INSTALL_DATADIR}/oppt/)

find_package(Doxygen)
if (DOXYGEN_FOUND)   
   set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)
   set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
   configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
   add_custom_target( doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
endif()

