set(ROBOT_ENVIRONMENT_SRC   
   ${CMAKE_CURRENT_LIST_DIR}/RobotEnvironment.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Body.cpp 
            ${CMAKE_CURRENT_LIST_DIR}/SceneImpl.cpp
            ${CMAKE_CURRENT_LIST_DIR}/geometric/Geometry.cpp
            ${CMAKE_CURRENT_LIST_DIR}/geometric/Mesh.cpp            
            ${CMAKE_CURRENT_LIST_DIR}/geometric/Box.cpp
            ${CMAKE_CURRENT_LIST_DIR}/geometric/Sphere.cpp
            ${CMAKE_CURRENT_LIST_DIR}/geometric/Cylinder.cpp
            ${CMAKE_CURRENT_LIST_DIR}/SDFParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CollisionObject.cpp)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/oppt/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)
