SET(ROBOTS_SRC
            ${CMAKE_CURRENT_LIST_DIR}/robot.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotState.cpp            
            ${CMAKE_CURRENT_LIST_DIR}/ActionSpace.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Action.cpp            
            ${CMAKE_CURRENT_LIST_DIR}/ContinuousActionSpace.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ContinuousVectorActionSpace.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Observation.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ObservationSpace.cpp
            ${CMAKE_CURRENT_LIST_DIR}/Serializer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotSerializer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotSDFParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotSpaceInformationFactory.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ActionSpaceDiscretizer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ObservationSpaceDiscretizer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/InverseKinematics/IKSolver.cpp)
if (SUPPORTS_IK)
    SET(ROBOTS_SRC ${ROBOTS_SRC} ${CMAKE_CURRENT_LIST_DIR}/InverseKinematics/InverseKinematics.cpp)
endif()
