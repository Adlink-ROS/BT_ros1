cmake_minimum_required(VERSION 2.8.3)
project(bt_sample)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
option(BUILD_OPENVINO "Build OpenVINO" OFF)
if(BUILD_OPENVINO STREQUAL "ON")
   find_package(catkin REQUIRED COMPONENTS
     actionlib
     actionlib_msgs
     behaviortree_cpp_v3
     roscpp
     std_msgs
     move_base_msgs
     object_msgs
   )
   add_definitions(-DSUPPORT_OPENVINO)
else()
   find_package(catkin REQUIRED COMPONENTS
     actionlib
     actionlib_msgs
     behaviortree_cpp_v3
     roscpp
     std_msgs
     move_base_msgs
   )
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS
   LIBRARIES BT_sample
   CATKIN_DEPENDS std_msgs actionlib actionlib_msgs behaviortree_cpp_v3 roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories( ${catkin_INCLUDE_DIRS})
# include_directories(/opt/sema/include/)

# link_directories(/opt/sema/binary/linux64/lib/)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node
   src/main.cpp
   src/movebase_client.cpp
   )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
