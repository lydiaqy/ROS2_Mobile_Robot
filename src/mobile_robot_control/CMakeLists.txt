cmake_minimum_required(VERSION 3.8)
project(mobile_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

add_executable(velocity_publisher src/velocity_publisher.cpp)

ament_target_dependencies(velocity_publisher
  rclcpp
  geometry_msgs
)

install(TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(velocity_subscriber
  src/velocity_subscriber.cpp
)

ament_target_dependencies(velocity_subscriber
  rclcpp
  geometry_msgs
)

install(TARGETS
  velocity_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(distance_controller
  src/distance_controller.cpp
)

ament_target_dependencies(distance_controller
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  distance_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(rotation_controller
  src/rotation_controller.cpp
)

ament_target_dependencies(rotation_controller
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  rotation_controller
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(square_path_controller
  src/square_path_controller.cpp
)

ament_target_dependencies(square_path_controller
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  square_path_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
