cmake_minimum_required (VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project (viewer VERSION 0.6.0)
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE DEBUG)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -O3 -DEIGEN_NO_DEBUG -w")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -DEIGEN_NO_DEBUG -w")
SET(BOOST_MIN_VERSION "1.54.0")
SET(PYTHON_MIN_VERSION "2.7.0")

include(FindPkgConfig)
find_package(catkin COMPONENTS bondcpp rviz REQUIRED)
find_package(Qt5Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

if (Qt5Widgets_FOUND)
  set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT_PATH}/../bin")
  add_executable(opptViewer
                 ${CMAKE_CURRENT_LIST_DIR}/main.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/RvizInterface.cpp) 
  target_include_directories(opptViewer PUBLIC ${catkin_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS})           
  target_link_libraries (opptViewer                     
                         Qt5::Widgets
                         ${catkin_LIBRARIES})
  install(TARGETS opptViewer DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
