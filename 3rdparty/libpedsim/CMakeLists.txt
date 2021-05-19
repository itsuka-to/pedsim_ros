cmake_minimum_required(VERSION 3.5)
project(pedsim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#------------------ Configuration ------------------#
option(SHALL_DEBUG "Enable debug features" ON)
option(SHALL_PROFILE "Enable the code profiling feature" OFF)
option(CMAKE_VERBOSE_MAKEFILE "Full compiler output" ON)

#debugging
if(SHALL_DEBUG)
  message("Debugging activated")
  add_definitions(-O0 -DDEBUG -ggdb -g3 -rdynamic)
else(SHALL_DEBUG)
  message("Debugging deactivated")
  add_definitions(-Os)
endif(SHALL_DEBUG)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS  roscpp
#   INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/
#   LIBRARIES pedsim
# )

include_directories(
  ${PROJECT_SOURCE_DIR}/include/pedsim/
  ${roscpp_INCLUDE_DIRS}
)

set(SOURCES
  src/ped_agent.cpp
  src/ped_angle.cpp
  src/ped_obstacle.cpp
  src/ped_scene.cpp
  src/ped_tree.cpp
  src/ped_vector.cpp
  src/ped_waypoint.cpp
)

add_library(pedsim SHARED ${SOURCES})

target_link_libraries(pedsim
  ${BOOST_LIBRARIES}
)

install(
  TARGETS pedsim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
