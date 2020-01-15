set(GAZEBO_INTERFACE_SRC 
            ${CMAKE_CURRENT_LIST_DIR}/gazebo/OpptODEPhysics.cpp
            ${CMAKE_CURRENT_LIST_DIR}/gazebo/OpptODELink.cpp
            ${CMAKE_CURRENT_LIST_DIR}/gazebo/World.cpp            
            ${CMAKE_CURRENT_LIST_DIR}/GazeboInterface.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ServerInterface.cpp
            ${CMAKE_CURRENT_LIST_DIR}/GazeboSubscriber.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sensors/SensorInterface.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sensors/GazeboSensor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sensors/RaySensor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sensors/GPSSensor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sensors/DepthSensor.cpp)
  set(GAZEBO_INTERFACE_SRC
    ${GAZEBO_INTERFACE_SRC}
    ${CMAKE_CURRENT_LIST_DIR}/SetSpaceInformationGZEight.cpp)
include_directories(${CMAKE_CURRENT_LIST_DIR}/gazebo/opende/include) 
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/oppt
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)
