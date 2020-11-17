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
            ${CMAKE_CURRENT_LIST_DIR}/ActionSpaceDiscretizer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/ObservationSpaceDiscretizer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/InverseKinematics/IKSolver.cpp)
if (SUPPORTS_IK)
    SET(ROBOTS_SRC ${ROBOTS_SRC} ${CMAKE_CURRENT_LIST_DIR}/InverseKinematics/InverseKinematics.cpp)
endif()

set(ROBOT_IMPL_SRC
    ${CMAKE_CURRENT_LIST_DIR}/RobotImpl/RobotImpl.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotImpl/RobotImplSDFParser.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotImpl/RobotImplSerializer.cpp
            ${CMAKE_CURRENT_LIST_DIR}/RobotImpl/RobotImplSpaceInformationFactory.cpp)
