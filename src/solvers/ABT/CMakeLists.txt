set(SOLVER_SRC_PATH "${CMAKE_CURRENT_LIST_DIR}/solver")
#include_directories(${CMAKE_CURRENT_LIST_DIR}/solver/)
include(${CMAKE_CURRENT_LIST_DIR}/robotModel/CMakeLists.txt)

file(GLOB_RECURSE SRCSABSTRACTPROBLEM ${CMAKE_CURRENT_LIST_DIR}/solver/abstract-problem/*.cpp)
file(GLOB_RECURSE SRCSBELIEFESTIMATORS ${CMAKE_CURRENT_LIST_DIR}/solver/belief-estimators/*.cpp)
file(GLOB_RECURSE SRCSOPTIONS ${CMAKE_CURRENT_LIST_DIR}/options/*.c*)
file(GLOB_RECURSE SRCSCHANGES ${CMAKE_CURRENT_LIST_DIR}/solver/changes/*.cpp)
file(GLOB_RECURSE SRCINDEXING ${CMAKE_CURRENT_LIST_DIR}/solver/indexing/*.cpp)
file(GLOB_RECURSE SRCMAPPINGS ${CMAKE_CURRENT_LIST_DIR}/solver/mappings/*.cpp)
file(GLOB_RECURSE SRCSEARCH ${CMAKE_CURRENT_LIST_DIR}/solver/search/*.cpp)
file(GLOB_RECURSE SRCSERIALIZATION ${CMAKE_CURRENT_LIST_DIR}/solver/serialization/*.cpp)

set(ABT_SRC
    ${SRCSABSTRACTPROBLEM}
    ${SRCSBELIEFESTIMATORS}
    ${SRCSOPTIONS}
    ${SRCSCHANGES}  
    ${SRCINDEXING}
    ${SRCMAPPINGS}
    ${SRCSEARCH}
    ${SRCSERIALIZATION}
    ${ROBOT_MODEL_SRC}
    ${SOLVER_SRC_PATH}/ActionNode.cpp
    ${SOLVER_SRC_PATH}/Agent.cpp 
    ${SOLVER_SRC_PATH}/BeliefNode.cpp
    ${SOLVER_SRC_PATH}/BeliefTree.cpp
    ${SOLVER_SRC_PATH}/Histories.cpp
    ${SOLVER_SRC_PATH}/HistoryEntry.cpp
    ${SOLVER_SRC_PATH}/HistorySequence.cpp
    ${SOLVER_SRC_PATH}/Simulator.cpp
    ${SOLVER_SRC_PATH}/Solver.cpp
    ${SOLVER_SRC_PATH}/StateInfo.cpp
    ${SOLVER_SRC_PATH}/StatePool.cpp)


if(NOT ${BLOCK_SOLVER_COMPILATION})
add_executable(abt
               ${CMAKE_CURRENT_LIST_DIR}/main_abt_robot.cpp
               ${ABT_SRC})
               
target_link_libraries (abt                   
                       oppt)
endif() 

