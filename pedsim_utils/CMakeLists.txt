cmake_minimum_required(VERSION 3.5)
project(pedsim_utils)

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

# set(PACKAGE_DEPS
#   cmake_modules
#   geometry_msgs
#   pedsim_msgs
#   roscpp
#   rospy
# )

# find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

find_package(ament_cmake REQUIRED)
# find_package(cmake_modules REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
# catkin_package(
#   CATKIN_DEPENDS ${PACKAGE_DEPS}
#   LIBRARIES ${PROJECT_NAME}
#   INCLUDE_DIRS
#     include
# )

###########
## Build ##
###########
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include)
include_directories(${ament_cmake_INCLUDE_DIRS})

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME}
  src/${PROJECT_NAME}/geometry.cpp
  src/${PROJECT_NAME}/pedsim_utils.cpp
)

ament_target_dependencies(${LIBRARY_NAME}
  geometry_msgs
  pedsim_msgs
  rclpy
  Eigen3
)

# add_dependencies(${LIBRARY_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(${LIBRARY_NAME}
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
  LIBRARY DESTINATION lib # ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include # ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pedsim_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
