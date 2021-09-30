cmake_minimum_required(VERSION 3.5)
project(packml_ros)

set(CMAKE_XX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(packml_msgs REQUIRED)
find_package(packml_sm REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

#include all directories
include_directories(
  include
)

add_executable(packml_ros_node src/packml_ros_node.cpp)
ament_target_dependencies(packml_ros_node rclcpp packml_msgs packml_sm std_msgs Qt5)

#install
install(TARGETS packml_ros_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_utest test/utest.cpp)
  ament_target_dependencies(${PROJECT_NAME}_utest rclcpp packml_msgs packml_sm std_msgs Qt5)
endif()

#Substituting the catkin_package () components:
ament_export_dependencies(packml_msgs packml_sm rclcpp std_msgs)

ament_package()
