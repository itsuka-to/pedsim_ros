cmake_minimum_required(VERSION 3.5)

project(pedsim_srvs)

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
find_package(pedsim_msgs REQUIRED)


set(srv_files
  "srv/GetAgentState.srv"
  "srv/GetAllAgentsState.srv"
  "srv/SetAgentState.srv"
  "srv/SetAllAgentsState.srv"
)

# the services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES pedsim_msgs std_msgs
  ADD_LINTER_TESTS  
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()