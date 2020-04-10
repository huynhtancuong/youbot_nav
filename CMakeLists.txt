cmake_minimum_required(VERSION 2.8.3)
project(youbot_nav)

find_package(catkin REQUIRED
  COMPONENTS
    robotics_math
    geometry_msgs
    kdl_parser
    roscpp
    roslint
    tf
    urdf
)

catkin_package(
  LIBRARIES
  robotics_math
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### LIBS
add_subdirectory(utils/robotics_math)

### TESTS


if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()


### INSTALLS
install(TARGETS robotics_math
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
