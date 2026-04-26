cmake_minimum_required(VERSION 3.5)
project(nav2_yolov7_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

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

# === Dependencies ===

find_package(nav2_costmap_2d REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util)
find_package(nav2_voxel_grid REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(perception2d_interfaces REQUIRED) 
find_package(angles REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    perception2d_interfaces
    nav_msgs
    )



include_directories(
  include
  ${perception2d_interfaces_INCLUDE_DIRS}
  )

# === Build ===

add_library(${lib_name} SHARED
            src/yolov7_layer.cpp)

ament_target_dependencies(${lib_name} ${dep_pkgs})
# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs yolov7_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d yolov7_layer.xml)

ament_package()
