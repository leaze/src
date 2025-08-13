cmake_minimum_required(VERSION 3.0.2)
project(astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  octomap_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp octomap_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/astar_3d.cpp
)

add_executable(astar_planner_node 
src/astar_planner_node.cpp
)
target_link_libraries(astar_planner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS astar_planner_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)