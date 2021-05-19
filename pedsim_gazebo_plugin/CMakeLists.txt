cmake_minimum_required(VERSION 3.5)
project(pedsim_gazebo_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)  # ignition needs C++17
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   gazebo_ros
#   geometry_msgs
#   roscpp
#   rospy
#   gazebo_msgs
#   pedsim_msgs
# )

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})


################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gzb_vel_plugin
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs pedsim_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(ActorPosesPlugin src/actor_poses_plugin.cpp)
target_link_libraries(ActorPosesPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(ActorPosesPlugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

