cmake_minimum_required(VERSION 2.8.3)
project(arm_warnings)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  tf
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arm_warnings
  CATKIN_DEPENDS moveit_ros_move_group moveit_ros_planning_interface roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${eigen3_INCLUDE_DIRS}
)

add_executable(arm_warnings src/arm_warnings.cpp)
target_link_libraries(arm_warnings ${catkin_LIBRARIES} ${eigen3_LIBRARIES} )
add_dependencies(arm_warnings ${catkin_EXPORTED_TARGETS})

