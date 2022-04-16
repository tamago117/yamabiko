cmake_minimum_required(VERSION 3.0.2)
project(yamabiko)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yamabiko
  CATKIN_DEPENDS roscpp rospy std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_to_pose   src/move_to_pose.cpp)
target_link_libraries(move_to_pose
  ${catkin_LIBRARIES}
)

add_executable(wpSelect   src/wpSelect.cpp)
target_link_libraries(wpSelect
  ${catkin_LIBRARIES}
)
