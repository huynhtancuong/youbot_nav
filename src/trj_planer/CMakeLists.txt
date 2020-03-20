cmake_minimum_required(VERSION 2.8.3)
project(youbot_nav)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    kdl_parser
    roscpp
    roslint
    tf
    urdf
)

catkin_package(
  LIBRARIES
    trj_planer
  CATKIN_DEPENDS
    Eigen
    spdlog
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### LIBS
add_library(trj_planer
  src/trj_planer/trj_planer.cpp
)
target_link_libraries(trj_planer
  ${catkin_LIBRARIES}
)

### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()


### INSTALLS
install(TARGETS trj_planer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
