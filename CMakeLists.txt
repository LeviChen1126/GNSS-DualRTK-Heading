cmake_minimum_required(VERSION 3.10)
project(gnss_dualrtk_heading)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(dualrtk_heading_node src/dualrtk_heading_node.cpp)
target_link_libraries(dualrtk_heading_node ${catkin_LIBRARIES})

install(TARGETS dualrtk_heading_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
