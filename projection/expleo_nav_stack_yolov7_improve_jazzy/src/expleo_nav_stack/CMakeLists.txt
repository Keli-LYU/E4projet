cmake_minimum_required(VERSION 3.8)
project(expleo_nav_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(expleo_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

include_directories(include)

set(executable_name expleo_planner_server)

set(dependencies
    # rclcpp_action
    # rclcpp_lifecycle
    # std_msgs
    # visualization_msgs
    # nav2_util
    # nav2_msgs
    # nav_msgs
    # geometry_msgs
    # builtin_interfaces
    # tf2_ros
    # nav2_costmap_2d
    # pluginlib
    # nav2_core
    rclcpp nav2_planner expleo_nav_msgs)

add_executable(${executable_name} src/expleo_planner_server.cpp src/main.cpp)

ament_target_dependencies(${executable_name} ${dependencies})

install(TARGETS ${executable_name} RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# BT plugin compilation setup #

set(dependencies rclcpp rclcpp_action rclcpp_lifecycle nav_msgs
                 behaviortree_cpp expleo_nav_msgs nav2_behavior_tree)

add_library(expleo_navstack_compute_local_plan_action_bt_node SHARED
            behavior_tree/plugins/action/compute_local_plan.cpp)

ament_target_dependencies(expleo_navstack_compute_local_plan_action_bt_node
                          ${dependencies})

target_compile_definitions(expleo_navstack_compute_local_plan_action_bt_node
                           PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS expleo_navstack_compute_local_plan_action_bt_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(expleo_navstack_compute_local_plan_action_bt_node)

# last command of the file #
ament_package()
