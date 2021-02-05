cmake_minimum_required(VERSION 3.5)
project(pe_ars408_ros)

if (NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif ()


if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(catkin
   REQUIRED COMPONENTS
   sensor_msgs
   can_msgs
   socketcan_bridge
   )


catkin_package(
   CATKIN_DEPENDS
   sensor_msgs
   can_msgs
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   include
)

# Driver Library
add_library(pe_ars408_parser
   src/ars408_parser.cpp
   )

# ROS node
add_executable(pe_ars408_node
   src/ars408_ros_node.cpp
   )

target_link_libraries(pe_ars408_node
   pe_ars408_parser
   ${catkin_LIBRARIES}
   )


install(TARGETS pe_ars408_node pe_ars408_parser
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   )