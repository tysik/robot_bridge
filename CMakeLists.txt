cmake_minimum_required(VERSION 2.8.3)
project(robot_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf)
find_package(Boost REQUIRED COMPONENTS system)

## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_bridge
  CATKIN_DEPENDS nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

#
# Build library
#
add_library(${PROJECT_NAME} src/robot_bridge.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#
# Build node
#
add_executable(${PROJECT_NAME}_node src/robot_bridge_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
