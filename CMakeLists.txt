cmake_minimum_required(VERSION 3.5)
project(pe_ars408_ros)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Driver Library
ament_auto_add_library(pe_ars408_parser SHARED
  src/ars408_driver.cpp
)

# ROS node
ament_auto_add_library(pe_ars408_node_component SHARED
  src/ars408_ros_node.cpp
)

target_link_libraries(pe_ars408_node_component
  pe_ars408_parser
)

rclcpp_components_register_node(pe_ars408_node_component
  PLUGIN "PeContinentalArs408Node"
  EXECUTABLE pe_ars408_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
