cmake_minimum_required(VERSION 3.5)
project(packml_plugin)

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
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(pluginlib REQUIRED)
find_package(packml_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
#find_package(class_loader REQUIRED)

#link_directories(${ament_cmake_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(QT_MOC include/packml_plugin/packml_plugin.h include/packml_plugin/packml_widget.h)
set(SOURCES src/packml_plugin.cpp src/packml_widget.cpp ${QT_MOC})

qt5_wrap_ui(QT_UI_FILES src/packml.ui)

#set_property(SOURCE include/plugin_panel.h PROPERTY SKIP_AUTOMOC ON)
 
add_library(${PROJECT_NAME} SHARED
  ${QT_UI_FILES}
  ${SOURCES}
)

include_directories(${PROJECT_NAME} 
  include/
)

target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES} rviz_common::rviz_common Qt5::Widgets Qt5::Core Qt5::Gui)
ament_target_dependencies(${PROJECT_NAME} "rviz_common" "Qt5" "packml_msgs" "std_msgs" "rclcpp" "rosidl_typesupport_cpp")

#target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
#target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

#ament_export_dependencies(rclcpp rviz_common Qt5)


#class_loader_hide_library_symbols(${PROJECT_NAME})

#install(FILES plugin_description.xml DESTINATION share)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

#install(DIRECTORY include)

#ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_libraries(${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

