add_executable(playVideo
               ${CMAKE_CURRENT_LIST_DIR}/playVideo.cpp)               
target_link_libraries (playVideo                    
                       oppt)
add_executable(showScene
               ${CMAKE_CURRENT_LIST_DIR}/ShowScene.cpp)               
target_link_libraries (showScene                
                       oppt)
