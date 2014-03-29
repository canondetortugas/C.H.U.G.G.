cmake_minimum_required(VERSION 2.8.3)
project(chugg_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs uscauv_common cpp11 dynamic_reconfigure)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/ICP.cfg 
  )

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES chugg_finder
  CATKIN_DEPENDS roscpp sensor_msgs uscauv_common cpp11 dynamic_reconfigure
  #  DEPENDS system_lib
)

add_executable(chugg_finder nodes/chugg_finder_node.cpp )
target_link_libraries(chugg_finder ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)