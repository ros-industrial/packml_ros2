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
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -fprofile-arcs -ftest-coverage)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(packml_msgs REQUIRED)
find_package(packml_sm REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

#include all directories
include_directories(include 
 ${rclcpp_INCLUDE_DIRS}
 ${packml_msgs_INCLUDE_DIRS}
 ${packml_sm_INCLUDE_DIRS}
 ${std_msgs_INCLUDE_DIRS}
 ${Qt5_INCLUDE_DIRS}
 /usr/include/gtest
 /usr/src/gtest
# /usr/include/gmock
# /usr/src/gmock
)

link_directories(/usr/lib)

add_executable(packml_ros_node src/packml_ros_node.cpp)
target_link_libraries(packml_ros_node ${rclcpp_LIBRARIES} ${packml_msgs_LIBRARIES} ${packml_sm_LIBRARIES} ${std_msgs_LIBRARIES} Qt5::Widgets Qt5::Core gcov)

add_executable(${PROJECT_NAME}_utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}_utest ${rclcpp_LIBRARIES} ${packml_msgs_LIBRARIES} ${packml_sm_LIBRARIES} ${std_msgs_LIBRARIES} Qt5::Widgets Qt5::Core pthread gtest gmock gcov)

#install
install(TARGETS packml_ros_node ${PROJECT_NAME}_utest
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  #ament_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  #target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} ${rclcpp_LIBRARIES} ${packml_msgs_LIBRARIES} ${packml_sm_LIBRARIES} ${std_msgs_LIBRARIES} Qt5::Widgets Qt5::Core pthread gtest)
endif()

#Substituting the catkin_package () components:
ament_export_dependencies(packml_msgs packml_sm rclcpp std_msgs)

ament_package()
