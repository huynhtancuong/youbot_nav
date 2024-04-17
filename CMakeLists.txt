cmake_minimum_required(VERSION 2.8.3)
project(youbot_nav)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    kdl_parser
    roscpp
    roslint
    tf
    urdf
    robotics_math
)

catkin_package(
  CATKIN_DEPENDS
  robotics_math
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## LIBS
add_library(point_motion
        src/motion/global_path_planer.h
        src/motion/global_path_planer.cpp
        src/motion/motion.h
        src/motion/motion.cpp
        src/slam/vectorized_laser_slam.h
        src/slam/vectorized_laser_slam.cpp
)
target_link_libraries(point_motion
  ${catkin_LIBRARIES}
  #  robotics_math
)

### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()


add_executable(test_global_trj_planer src/tests/ros_global_trj_planer_test.cpp)
target_link_libraries(test_global_trj_planer point_motion ${catkin_LIBRARIES})

add_executable(test_global_motion src/tests/ros_motion_test.cpp)
target_link_libraries(test_global_motion point_motion ${catkin_LIBRARIES})

add_executable(test_pid_controller src/tests/ros_pid_control_test.cpp)
target_link_libraries(test_pid_controller point_motion ${catkin_LIBRARIES})

add_executable(test_global_motion_control src/tests/ros_motion_control_test.cpp)
target_link_libraries(test_global_motion_control point_motion ${catkin_LIBRARIES})


install(PROGRAMS scripts/test_nav1.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
