cmake_minimum_required(VERSION 3.5)

project(pedsim_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/AgentState.msg"
  "msg/AgentStates.msg"
  "msg/AgentGroup.msg"
  "msg/AgentGroups.msg"
  "msg/AgentForce.msg"
  "msg/LineObstacle.msg"
  "msg/LineObstacles.msg"
  "msg/TrackedPerson.msg"
  "msg/TrackedPersons.msg"
  "msg/TrackedGroup.msg"
  "msg/TrackedGroups.msg"
  "msg/SocialRelation.msg"
  "msg/SocialRelations.msg"
  "msg/SocialActivity.msg"
  "msg/SocialActivities.msg"
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()