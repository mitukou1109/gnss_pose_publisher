cmake_minimum_required(VERSION 3.8)
project(gnss_pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_pose_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "gnss_pose_publisher::GNSSPosePublisher"
  EXECUTABLE gnss_pose_publisher_node
)

ament_auto_package()
