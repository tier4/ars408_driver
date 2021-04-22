cmake_minimum_required(VERSION 3.5)
project(pe_ars408_ros)

if (NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif ()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(Boost REQUIRED)

# Driver Library
include_directories(include/)

add_library(pe_ars408_parser
   src/ars408_driver.cpp
)

target_include_directories(pe_ars408_parser PRIVATE
   ${Boost_INCLUDE_DIRS}
)

target_link_libraries(pe_ars408_parser PRIVATE
        ${Boost_LIBRARIES}
)

# ROS node
add_executable(pe_ars408_node
   src/ars408_ros_node.cpp
)
ament_target_dependencies(pe_ars408_node
   rclcpp autoware_perception_msgs sensor_msgs can_msgs unique_identifier_msgs
)

target_link_libraries(pe_ars408_node
   pe_ars408_parser
)

install(TARGETS pe_ars408_node pe_ars408_parser
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.launch.xml"
)

ament_package()