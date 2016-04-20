cmake_minimum_required(VERSION 2.8.3)
project(pr2_movement_routines)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  simple_robot_control
  std_msgs
  roscpp
  rospy
  pr2_lfd_utils
  tf
  moveit_core
  pr2_controllers_msgs
  moveit_msgs
  moveit_ros_planning_interface
  trajectory_msgs
  particle_filter
)

find_package(Eigen REQUIRED)

find_package(Boost REQUIRED)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS simple_robot_control pr2_controllers_msgs trajectory_msgs std_msgs roscpp tf geometry_msgs pr2_lfd_utils particle_filter boost moveit_core moveit_ros_planning_interface moveit_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)



add_executable(gazebo_routines nodes/gazebo_test.cpp)
add_executable(tester nodes/testfile.cpp)
target_link_libraries(gazebo_routines ${catkin_LIBRARIES} ${particle_filter_LIBRARY})
target_link_libraries(tester ${catkin_LIBRARIES})
add_dependencies(gazebo_routines ${PROJECT_NAME}_gencpp ${GENCPP_DEPS} particle_filter)
add_dependencies(tester ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

#############
## Install ##
#############

install(PROGRAMS
  scripts/arWorldModel.py
  scripts/recordInteraction.py
  scripts/bag2mat.py
  scripts/extractMarkers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS ${ROUTINES_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

