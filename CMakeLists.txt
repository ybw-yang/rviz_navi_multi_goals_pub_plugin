cmake_minimum_required(VERSION 3.16)
project(navi_multi_goals_pub_rviz_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# Share header contracts with navigation without linking its libraries.
find_path(VIRTUAL_WALL_GEOMETRY_INCLUDE_DIR virtual_wall_geometry.h
  HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../general_algorithm/map_system/include)
find_path(VIRTUAL_WALL_RECEIVER_INCLUDE_DIR
  behavior_tree/plugins/action/dataReceiver/virtual_wall_ros.h
  HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../general_algorithm/pathplanning_behavior_tree/include)
if(NOT VIRTUAL_WALL_GEOMETRY_INCLUDE_DIR OR NOT VIRTUAL_WALL_RECEIVER_INCLUDE_DIR)
  message(FATAL_ERROR "Virtual wall headers not found. Set VIRTUAL_WALL_GEOMETRY_INCLUDE_DIR and VIRTUAL_WALL_RECEIVER_INCLUDE_DIR to their source or installed include directories.")
endif()

add_library(${PROJECT_NAME} SHARED
  src/multi_navi_goal_panel.cpp
  include/multi_navi_goal_panel.hpp
  src/point_cloud_goal_tool.cpp
  include/point_cloud_goal_tool.hpp
  src/virtual_wall_editor.cpp
  include/navi_multi_goals_pub_rviz_plugin/virtual_wall_editor.hpp
  src/height_display.cpp
  include/navi_multi_goals_pub_rviz_plugin/height_display.hpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include
  ${VIRTUAL_WALL_GEOMETRY_INCLUDE_DIR} ${VIRTUAL_WALL_RECEIVER_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp tf2 tf2_geometry_msgs rviz_common rviz_rendering rviz_default_plugins
  std_msgs visualization_msgs geometry_msgs sensor_msgs pcl_conversions pluginlib
)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include/)
install(FILES README.md VIRTUAL_WALL.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(virtual_wall_protocol_test test/virtual_wall_protocol_test.cpp)
  target_include_directories(virtual_wall_protocol_test PRIVATE
    ${VIRTUAL_WALL_GEOMETRY_INCLUDE_DIR} ${VIRTUAL_WALL_RECEIVER_INCLUDE_DIR})
  ament_target_dependencies(virtual_wall_protocol_test visualization_msgs)
  target_link_libraries(virtual_wall_protocol_test Eigen3::Eigen)
  ament_add_gtest(cloud_height_filter_test test/height_filter_test.cpp)
  target_include_directories(cloud_height_filter_test PRIVATE include)
  ament_target_dependencies(cloud_height_filter_test sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(Qt5 Eigen3 PCL rviz_common rviz_default_plugins rviz_rendering
  geometry_msgs sensor_msgs visualization_msgs std_msgs rclcpp tf2 tf2_geometry_msgs pcl_conversions pluginlib)
ament_package()
