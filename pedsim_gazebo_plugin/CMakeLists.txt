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
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(std_msgs REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
# include_directories(include)
include_directories(${ament_cmake_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ActorPosesPlugin SHARED src/actor_poses_plugin.cpp)

target_link_libraries(ActorPosesPlugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(ActorPosesPlugin 
  gazebo
  rclcpp
  std_msgs
  pedsim_msgs
)

install(
  TARGETS ActorPosesPlugin
  DESTINATION ${PROJECT_NAME}
)

ament_python_install_package(scripts)
install(PROGRAMS
  scripts/pedsim_to_gazebo_world.py
  scripts/spawn_pedsim_agents.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

#############
## Install ##
#############

