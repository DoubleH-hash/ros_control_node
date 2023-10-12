cmake_minimum_required(VERSION 3.0.2)
project(ys_ros_conveyor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ys_ros_msgs
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
# INCLUDE_DIRS include
#  LIBRARIES iau_ros_bridge
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories(
  include
  src
  UART
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(UART
  UART/uart_driver.cpp
  UART/uart_driver.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ys_ros_conveyor src/main.cpp)
add_dependencies(ys_ros_conveyor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ys_ros_conveyor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(UART ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
target_link_libraries(UART
  ${catkin_LIBRARIES}
)
target_link_libraries(ys_ros_conveyor
  ${catkin_LIBRARIES}
)
target_link_libraries(ys_ros_conveyor
  UART
  ${catkin_LIBRARIES}
)

