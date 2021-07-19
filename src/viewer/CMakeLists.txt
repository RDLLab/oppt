find_package(catkin COMPONENTS bondcpp rviz REQUIRED)
find_package(Qt5Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

if (Qt5Widgets_FOUND)  
  add_executable(opptViewer
                 ${CMAKE_CURRENT_LIST_DIR}/main.cpp
                 ${CMAKE_CURRENT_LIST_DIR}/RvizInterface.cpp) 
  target_include_directories(opptViewer PUBLIC ${catkin_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}) 
  set_target_properties(opptViewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_BINDIR})          
  target_link_libraries (opptViewer                     
                         Qt5::Widgets
                         ${catkin_LIBRARIES})  
  install(TARGETS opptViewer DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
