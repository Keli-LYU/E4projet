cmake_minimum_required(VERSION 3.8)
project(local_planner_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(expleo_nav_stack REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(library template_local_planner_plugin)

set(dependencies rclcpp expleo_nav_stack)

set(sources src/template_local_planner.cpp)

add_library(${library} SHARED ${sources})
ament_target_dependencies(${library} ${dependencies})
target_compile_definitions(${library} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${library} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(FILES local_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(expleo_nav_stack local_planner_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${library})
ament_export_dependencies(${dependencies})


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
