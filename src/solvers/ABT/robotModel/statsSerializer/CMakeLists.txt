add_executable(statsSerializer
               ${CMAKE_CURRENT_LIST_DIR}/main_serializer.cpp               
               ${ROBOT_PROBLEM_SRC}
               ${DUBIN_SRC}
               ${ROBOT_IMPL_SRC}) 

target_link_libraries (statsSerializer                     
                       ${COMMON_LIBS})
