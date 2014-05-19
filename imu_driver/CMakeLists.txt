cmake_minimum_required(VERSION 2.8.3)
project(imu_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS uscauv_common tf)

include_directories(include include/imu_driver ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES imu_driver
CATKIN_DEPENDS uscauv_common tf
#  DEPENDS system_lib
)

add_executable( imu_driver nodes/imu_driver_node.cpp src/vn100.c src/vncp_services.c)
target_link_libraries(imu_driver ${catkin_LIBRARIES} rt pthread)