include(${CMAKE_CURRENT_LIST_DIR}/../shared/RRTConnect/CMakeLists.txt)
set(RRT_HEURISTIC_PLUGIN_SRC
    ${CMAKE_CURRENT_LIST_DIR}/ManipulatorRRTHeuristicPlugin.cpp
    ${RRT_CONNECT_SRC})
ADD_HEURISTIC_PLUGIN(ManipulatorRRTHeuristicPlugin "${RRT_HEURISTIC_PLUGIN_SRC}")
