SET(FILTER_SRC ${CMAKE_CURRENT_LIST_DIR}/particleFilter/ParticleFilter.cpp)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/particleFilter/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/oppt/filter/particleFilter
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.hpp"
  PATTERN ".DS_Store" EXCLUDE
)
