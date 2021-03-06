cmake_minimum_required(VERSION 2.8.3)
project(chugg_tracker)
set(CMAKE_BUILD_TYPE Debug)
# set( CMAKE_VERBOSE_MAKEFILE on )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs uscauv_common cpp11 dynamic_reconfigure ar_track_alvar message_generation message_runtime geometry_msgs)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED bfl)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BFL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_message_files(FILES
  Sample.msg
  Posterior.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ChuggFilter.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chugg_filter
  CATKIN_DEPENDS roscpp sensor_msgs uscauv_common cpp11 dynamic_reconfigure ar_track_alvar message_generation message_runtime geometry_msgs
  #  DEPENDS system_lib
)

add_library(chugg_filter src/chugg_filter.cpp src/marker_measurement_pdf.cpp 
  src/system_pdf_constant_velocity.cpp src/imu_measurement_pdf.cpp src/quaternion.cpp)
target_link_libraries(chugg_filter ${catkin_LIBRARIES} ${BFL_LDFLAGS} ${Eigen_LIBRARIES})
add_dependencies(chugg_filter ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
add_executable( chugg_tracker nodes/chugg_tracker_node.cpp )
target_link_libraries(chugg_tracker ${catkin_LIBRARIES} chugg_filter ${BFL_LDFLAGS} ${Eigen_LIBRARIES})

# Auto-generated by uscauv-add-node
# add_executable( include_test nodes/include_test_node.cpp )
# message("BFL LIBRARIES " ${BFL_LDFLAGS})
# target_link_libraries(include_test ${catkin_LIBRARIES} ${BFL_LDFLAGS})
