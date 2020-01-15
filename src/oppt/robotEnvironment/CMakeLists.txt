set(ROBOT_ENVIRONMENT_SRC   
   ${CMAKE_CURRENT_LIST_DIR}/RobotEnvironment.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Body.cpp 
            ${CMAKE_CURRENT_LIST_DIR}/SceneImpl.cpp          
            ${CMAKE_CURRENT_LIST_DIR}/BoxBody.cpp
            ${CMAKE_CURRENT_LIST_DIR}/SphereBody.cpp
            ${CMAKE_CURRENT_LIST_DIR}/MeshBody.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CylinderBody.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Mesh.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Box.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Sphere.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Cylinder.cpp
            ${CMAKE_CURRENT_LIST_DIR}/SDFParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/CollisionObject.cpp)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/oppt/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)
