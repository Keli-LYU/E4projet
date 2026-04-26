cmake_minimum_required(VERSION 3.8)
project(carla_twist_to_regulated_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list of all message packages that have to be found and added as dependencies
set(MESSAGES_PKGS std_msgs carla_msgs geometry_msgs nav_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

foreach(msgs ${MESSAGES_PKGS})
    find_package(${msgs} REQUIRED)
endforeach()

include_directories(include)

set(SOURCES src/TwistToRegulatedControl.cpp         
            src/PIDController.cpp                   
            src/twist_to_regulated_control_node.cpp
    )

add_executable(twist_to_regulated_control ${SOURCES})
ament_target_dependencies(twist_to_regulated_control rclcpp ${MESSAGES_PKGS})

# install target
install(TARGETS
    twist_to_regulated_control
    DESTINATION lib/${PROJECT_NAME})

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
