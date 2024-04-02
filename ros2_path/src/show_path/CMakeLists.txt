cmake_minimum_required(VERSION 3.8)
project(show_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(Serial)
find_package(show_path_interfaces REQUIRED)


# add_executable(talker src/txtPath.cpp)
# add_executable(talker src/txtControl.cpp src/Serial.cpp)
add_executable(talker src/SerialControl.cpp src/Serial.cpp)
ament_target_dependencies(talker rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs show_path_interfaces)

add_executable(server src/MC_server.cpp)
ament_target_dependencies(server rclcpp show_path_interfaces)

add_executable(client src/async_client.cpp)
ament_target_dependencies(client rclcpp show_path_interfaces)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS
  talker
  server
  client
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
